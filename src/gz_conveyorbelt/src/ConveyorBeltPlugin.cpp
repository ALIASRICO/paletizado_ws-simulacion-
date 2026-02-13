/*
 * Conveyor Belt Plugin for Gazebo Sim 8 (Harmonic)
 * 
 * Este plugin detecta objetos sobre la cinta transportadora y les aplica
 * fuerzas externas (ExternalWorldWrenchCmd) para simular el movimiento.
 * 
 * Licensed under Apache-2.0
 */

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/transport/Node.hh>

#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace gz_conveyorbelt
{

/// \brief Conveyor Belt Plugin - Applies forces to objects on the belt
class ConveyorBelt
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  ConveyorBelt() = default;

  /// \brief Destructor
  ~ConveyorBelt() override = default;

  /// \brief Configure from SDF
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->modelEntity = _entity;
    auto model = gz::sim::Model(_entity);

    std::cout << "=== ConveyorBelt plugin loaded for: "
              << model.Name(_ecm) << " ===" << std::endl;

    // Read SDF parameters
    if (_sdf->HasElement("speed"))
    {
      this->speed = _sdf->Get<double>("speed");
    }

    if (_sdf->HasElement("link_name"))
    {
      this->beltLinkName = _sdf->Get<std::string>("link_name");
    }

    if (_sdf->HasElement("direction"))
    {
      this->direction = _sdf->Get<gz::math::Vector3d>("direction");
      this->direction.Normalize();
    }

    if (_sdf->HasElement("belt_length"))
    {
      this->beltLength = _sdf->Get<double>("belt_length");
    }

    if (_sdf->HasElement("belt_width"))
    {
      this->beltWidth = _sdf->Get<double>("belt_width");
    }

    // Find belt link entity
    this->beltLinkEntity = model.LinkByName(_ecm, this->beltLinkName);

    if (this->beltLinkEntity == gz::sim::kNullEntity)
    {
      std::cerr << "ERROR: Link [" << this->beltLinkName
                << "] NOT FOUND!" << std::endl;
      return;
    }

    // Subscribe to control topic
    std::string topic = "/conveyor/cmd_vel";
    if (_sdf->HasElement("topic"))
    {
      topic = _sdf->Get<std::string>("topic");
    }

    this->node.Subscribe(topic, &ConveyorBelt::OnSpeedCmd, this);

    std::cout << "Belt link found: " << this->beltLinkName << std::endl;
    std::cout << "Speed: " << this->speed << " m/s" << std::endl;
    std::cout << "Direction: " << this->direction << std::endl;
    std::cout << "Belt size: " << this->beltLength << " x "
              << this->beltWidth << " m" << std::endl;
    std::cout << "Listening on: " << topic << std::endl;
    std::cout << "Usage: gz topic -t " << topic
              << " -m gz.msgs.Double -p 'data: 0.5'" << std::endl;

    this->configured = true;
  }

  /// \brief PreUpdate - Apply forces to objects on belt
  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    // Skip if not configured or paused
    if (!this->configured || _info.paused)
    {
      return;
    }

    if (this->beltLinkEntity == gz::sim::kNullEntity)
    {
      return;
    }

    // First cycle: enable necessary components
    if (!this->initialized)
    {
      this->InitializeComponents(_ecm);
      this->initialized = true;
      return;  // Wait one cycle for components to be ready
    }

    double currentSpeed;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      currentSpeed = this->speed;
    }

    // Skip if speed is essentially zero
    if (std::abs(currentSpeed) < 1e-6)
    {
      return;
    }

    // Get belt link WORLD pose using gz::sim::worldPose function
    gz::math::Pose3d beltWorldPose = gz::sim::worldPose(this->beltLinkEntity, _ecm);
    double beltZ = beltWorldPose.Pos().Z();
    double beltX = beltWorldPose.Pos().X();
    double beltY = beltWorldPose.Pos().Y();

    double halfL = this->beltLength / 2.0;
    double halfW = this->beltWidth / 2.0;

    // Debug: print belt position every 500 iterations
    static int beltDebugCounter = 0;
    if (beltDebugCounter++ % 500 == 0)
    {
      std::cout << "[ConveyorBelt] Belt world pose: X=" << beltX
                << " Y=" << beltY << " Z=" << beltZ << std::endl;
    }

    // Iterate over all links in simulation
    _ecm.Each<gz::sim::components::Link>(
        [&](const gz::sim::Entity &_linkEntity,
            const gz::sim::components::Link *) -> bool
        {
          // Skip belt link itself
          if (_linkEntity == this->beltLinkEntity)
          {
            return true;
          }

          // Skip links belonging to conveyor model
          auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(
              _linkEntity);
          if (parentComp && parentComp->Data() == this->modelEntity)
          {
            return true;
          }

          // Skip static entities
          if (parentComp)
          {
            auto staticComp = _ecm.Component<gz::sim::components::Static>(
                parentComp->Data());
            if (staticComp && staticComp->Data())
            {
              return true;
            }
          }

          // Get link WORLD pose
          gz::math::Pose3d linkWorldPose = gz::sim::worldPose(_linkEntity, _ecm);
          const gz::math::Vector3d &pos = linkWorldPose.Pos();

          // Get link name for debug
          auto nameComp = _ecm.Component<gz::sim::components::Name>(_linkEntity);
          std::string linkName = nameComp ? nameComp->Data() : "unknown";

          // Debug: print all link positions every 500 iterations
          static int linkDebugCounter = 0;
          if (linkDebugCounter++ % 500 == 0)
          {
            std::cout << "[ConveyorBelt] Checking link '" << linkName
                      << "' at pos: X=" << pos.X() << " Y=" << pos.Y()
                      << " Z=" << pos.Z() << std::endl;
          }

          // Check if object is ON the belt (AABB check)
          // Belt surface is at beltZ + 0.01 (half of belt thickness)
          double beltSurfaceZ = beltZ + 0.01;
          bool onBelt =
              (pos.X() > beltX - halfL) && (pos.X() < beltX + halfL) &&
              (pos.Y() > beltY - halfW) && (pos.Y() < beltY + halfW) &&
              (pos.Z() > beltSurfaceZ - 0.05) && (pos.Z() < beltSurfaceZ + 0.5);

          if (onBelt)
          {
            // Get link mass from Inertial component
            auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(
                _linkEntity);

            double mass = 1.0; // Default mass
            if (inertialComp)
            {
              mass = inertialComp->Data().MassMatrix().Mass();
            }

            // Calculate force using proportional control
            double forceMagnitude = mass * currentSpeed * 15.0;
            gz::math::Vector3d force = this->direction * forceMagnitude;

            // Get current velocity for feedback control
            auto velComp = _ecm.Component<
                gz::sim::components::LinearVelocity>(_linkEntity);

            double currentVelInDir = 0.0;
            if (velComp)
            {
              currentVelInDir = velComp->Data().Dot(this->direction);
              double velError = currentSpeed - currentVelInDir;

              // Proportional control with gain
              forceMagnitude = mass * velError * 30.0;

              // Limit maximum force
              double maxForce = mass * 80.0;
              forceMagnitude = std::max(-maxForce,
                                        std::min(maxForce, forceMagnitude));

              force = this->direction * forceMagnitude;
            }

            // Debug output (only every 100 iterations to avoid spam)
            static int debugCounter = 0;
            if (debugCounter++ % 100 == 0)
            {
              std::cout << "[ConveyorBelt] Object '" << linkName
                        << "' on belt! Mass=" << mass
                        << " kg, Vel=" << currentVelInDir
                        << " m/s, Force=" << force.X() << " N" << std::endl;
            }

            // Apply external wrench (force)
            gz::msgs::Wrench wrenchMsg;
            wrenchMsg.mutable_force()->set_x(force.X());
            wrenchMsg.mutable_force()->set_y(force.Y());
            wrenchMsg.mutable_force()->set_z(0); // No vertical force
            wrenchMsg.mutable_torque()->set_x(0);
            wrenchMsg.mutable_torque()->set_y(0);
            wrenchMsg.mutable_torque()->set_z(0);

            auto wrenchComp = _ecm.Component<
                gz::sim::components::ExternalWorldWrenchCmd>(_linkEntity);

            if (!wrenchComp)
            {
              _ecm.CreateComponent(_linkEntity,
                                   gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
            }
            else
            {
              *wrenchComp = gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg);
            }
          }

          return true;
        });
  }

private:
  /// \brief Initialize Pose and LinearVelocity components
  void InitializeComponents(gz::sim::EntityComponentManager &_ecm)
  {
    std::cout << "ConveyorBelt: Initializing velocity checks..." << std::endl;

    // Enable Pose on the belt
    if (!_ecm.Component<gz::sim::components::Pose>(this->beltLinkEntity))
    {
      _ecm.CreateComponent(this->beltLinkEntity,
                           gz::sim::components::Pose());
    }

    // Enable velocity checks on all links
    _ecm.Each<gz::sim::components::Link>(
        [&](const gz::sim::Entity &_linkEntity,
            const gz::sim::components::Link *) -> bool
        {
          auto link = gz::sim::Link(_linkEntity);
          link.EnableVelocityChecks(_ecm, true);

          // Ensure Pose exists
          if (!_ecm.Component<gz::sim::components::Pose>(_linkEntity))
          {
            _ecm.CreateComponent(_linkEntity,
                                 gz::sim::components::Pose());
          }

          return true;
        });

    std::cout << "ConveyorBelt: Initialization complete." << std::endl;
  }

  /// \brief Callback for speed commands
  void OnSpeedCmd(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->speed = _msg.data();
    std::cout << "[ConveyorBelt] Speed changed to: " << this->speed
              << " m/s" << std::endl;
  }

  // Member variables
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity beltLinkEntity{gz::sim::kNullEntity};
  std::string beltLinkName{"belt_link"};
  double speed{0.5};
  gz::math::Vector3d direction{1, 0, 0};
  double beltLength{2.0};
  double beltWidth{0.5};
  gz::transport::Node node;
  std::mutex mutex;
  bool configured{false};
  bool initialized{false};
};

}  // namespace gz_conveyorbelt

// Register the plugin with Gazebo
GZ_ADD_PLUGIN(
    gz_conveyorbelt::ConveyorBelt,
    gz::sim::System,
    gz_conveyorbelt::ConveyorBelt::ISystemConfigure,
    gz_conveyorbelt::ConveyorBelt::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz_conveyorbelt::ConveyorBelt,
    "gz_conveyorbelt::ConveyorBelt")
