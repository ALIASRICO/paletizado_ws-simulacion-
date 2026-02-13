# =============================================================================
# Dockerfile Multi-Stage - Sistema Completo DOBOT CR20V + YOLO + ROS2 Jazzy
# =============================================================================
# 
# Este Dockerfile crea un entorno completo para:
#   - Robot DOBOT CR20V (20kg payload)
#   - Simulación Gazebo Sim 8 (Harmonic)
#   - Detección YOLOv8-OBB
#   - MoveIt2 para planificación de movimiento
#   - Plugin de banda transportadora personalizado
#
# Build:
#   docker build -t dobot-cr20v-complete:latest -f docker/Dockerfile .
#
# Run:
#   docker run -it --rm dobot-cr20v-complete:latest
#
# =============================================================================

# =============================================================================
# STAGE 1: BUILDER - Compilación de todos los paquetes ROS2
# =============================================================================
FROM ros:jazzy-perception AS builder

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias de build
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Gazebo development packages
    libgz-sim8-dev \
    libgz-plugin2-dev \
    libgz-common5-dev \
    libgz-math7-dev \
    libgz-transport13-dev \
    libgz-cmake3-dev \
    libgz-msgs10-dev \
    # ROS2 Gazebo integration
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    # MoveIt2 dependencies
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-configs-utils \
    # Control dependencies
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-xacro \
    # Additional ROS2 packages
    ros-jazzy-vision-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    # Python dependencies
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
WORKDIR /ros2_ws

# Copiar TODOS los package.xml primero (para cache de Docker)
# Paquetes DOBOT CR20V
COPY src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_msgs_v4/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/
COPY src/DOBOT_6Axis_ROS2_V4/cr20_moveit/package.xml src/DOBOT_6Axis_ROS2_V4/cr20_moveit/
COPY src/DOBOT_6Axis_ROS2_V4/cra_description/package.xml src/DOBOT_6Axis_ROS2_V4/cra_description/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_moveit/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_moveit/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_vision/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_vision/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_demo/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_demo/
COPY src/DOBOT_6Axis_ROS2_V4/dobot_rviz/package.xml src/DOBOT_6Axis_ROS2_V4/dobot_rviz/
COPY src/DOBOT_6Axis_ROS2_V4/servo_action/package.xml src/DOBOT_6Axis_ROS2_V4/servo_action/
# Paquetes personalizados
COPY src/gz_conveyorbelt/package.xml src/gz_conveyorbelt/
COPY src/gz_conveyorbelt/CMakeLists.txt src/gz_conveyorbelt/
COPY src/pruebas_de_vision/package.xml src/pruebas_de_vision/
COPY src/pruebas_de_vision/setup.py src/pruebas_de_vision/

# Instalar dependencias ROS2 con rosdep
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "ultralytics roboflow gz-msgs10"

# Copiar TODO el código fuente
COPY src/ src/

# Build de TODOS los paquetes ROS2
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --continue-on-error"

# =============================================================================
# Crear entorno virtual Python para YOLO (aislado de cv_bridge)
# =============================================================================
RUN python3 -m venv /opt/yolo_venv && \
    /opt/yolo_venv/bin/pip install --upgrade pip && \
    /opt/yolo_venv/bin/pip install \
    "ultralytics>=8.0.0" \
    "numpy<2" \
    "opencv-python-headless>=4.8.0" \
    "roboflow>=1.1.0" \
    "pyyaml>=6.0" \
    "matplotlib>=3.7.0"

# =============================================================================
# STAGE 2: RUNTIME - Imagen de producción completa
# =============================================================================
FROM ros:jazzy-perception AS runtime

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Instalar todas las dependencias de runtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Runtime essentials
    python3-pip \
    python3-opencv \
    # Gazebo runtime
    gz-sim8 \
    gz-plugin2 \
    gz-common5 \
    gz-math7 \
    gz-transport13 \
    gz-msgs10 \
    # ROS2 Gazebo bridge
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    # MoveIt2 runtime
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-planners \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-configs-utils \
    # Control runtime
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-xacro \
    # Additional ROS2 packages
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-vision-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-default-plugins \
    # X11 for GUI
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxext6 \
    libxrender1 \
    # Network tools
    iputils-ping \
    net-tools \
    # Cleanup
    && rm -rf /var/lib/apt/lists/*

# Crear usuario no-root para seguridad
RUN useradd -m -s /bin/bash rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Crear directorio de trabajo
WORKDIR /ros2_ws

# Copiar workspace compilado desde builder
COPY --from=builder /ros2_ws/install /ros2_ws/install
COPY --from=builder /ros2_ws/build /ros2_ws/build
COPY --from=builder /ros2_ws/log /ros2_ws/log

# Copiar entorno virtual de YOLO
COPY --from=builder /opt/yolo_venv /opt/yolo_venv

# Copiar código fuente completo (para symlink-install)
COPY src/ src/

# Crear directorios para datos y modelos
RUN mkdir -p /ros2_ws/runs /ros2_ws/data /ros2_ws/models && \
    chown -R rosuser:rosuser /ros2_ws

# =============================================================================
# Variables de entorno
# =============================================================================
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_SIM_RESOURCE_PATH=/ros2_ws/src/gz_conveyorbelt/worlds:/ros2_ws/src/pruebas_de_vision/worlds:/ros2_ws/src/DOBOT_6Axis_ROS2_V4/dobot_gazebo/worlds
ENV GZ_PLUGIN_PATH=/ros2_ws/install/gz_conveyorbelt/lib:$GZ_PLUGIN_PATH
ENV PATH="/opt/yolo_venv/bin:$PATH"
ENV VIRTUAL_ENV="/opt/yolo_venv"

# Copiar entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Cambiar a usuario no-root
USER rosuser

# Source ROS2 workspace en bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/yolo_venv/bin/activate 2>/dev/null || true" >> ~/.bashrc

# Exponer puertos
# Gazebo
EXPOSE 11345 7400-7500
# ROS2 DDS (para comunicación externa)
# EXPOSE 7400-7500/udp

# Healthcheck
HEALTHCHECK --interval=30s --timeout=10s --start-period=120s --retries=3 \
    CMD ros2 topic list > /dev/null 2>&1 || exit 1

# Entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
