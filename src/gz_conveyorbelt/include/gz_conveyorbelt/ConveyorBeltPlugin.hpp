/*
 * Conveyor Belt Plugin for Gazebo Sim 8 (Harmonic)
 * 
 * Este plugin detecta objetos sobre la cinta transportadora y les aplica
 * velocidad lineal para simular el movimiento por fricción.
 * 
 * Basado en la recomendación de usar LinearVelocityCmd para objetos
 * en contacto con la superficie de la cinta.
 * 
 * Licensed under Apache-2.0
 */

#ifndef GZ_CONVEYORBELT_PLUGIN_HPP_
#define GZ_CONVEYORBELT_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <memory>

namespace gz_conveyorbelt
{

/// \brief Private implementation class
class ConveyorBeltPrivate;

/// \brief A plugin that simulates a conveyor belt by detecting objects
/// on top of the belt surface and applying linear velocity to them.
///
/// This plugin:
/// 1. Reads belt dimensions and speed from SDF parameters
/// 2. Each update, checks all dynamic links in the simulation
/// 3. If a link is within the belt's bounding box, applies velocity
/// 4. Velocity is applied in the belt's local X direction
///
/// SDF Parameters:
/// - <link_name>: Name of the belt link (default: "belt_link")
/// - <speed>: Belt speed in m/s (default: 0.5)
/// - <direction>: Movement direction vector (default: "1 0 0")
/// - <topic>: Control topic (default: "/conveyor/cmd_vel")
/// - <belt_length>: Length of belt in X direction (default: 2.0)
/// - <belt_width>: Width of belt in Y direction (default: 0.5)
class ConveyorBelt
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  /// \brief Constructor
  ConveyorBelt();

  /// \brief Destructor
  ~ConveyorBelt() override;

  /// \brief Configure the plugin from SDF parameters
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  /// \brief Pre-update callback - applies velocity to objects on belt
  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

private:
  /// \brief Private data pointer (PIMPL pattern)
  std::unique_ptr<ConveyorBeltPrivate> dataPtr;
};

}  // namespace gz_conveyorbelt

#endif  // GZ_CONVEYORBELT_PLUGIN_HPP_
