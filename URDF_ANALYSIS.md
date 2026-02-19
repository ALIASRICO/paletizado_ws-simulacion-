# Análisis del URDF - Robot CR20

**Archivo**: `/home/iudc/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf`

**Robot Name**: cr20_robot

**Total Links**: 8

**Total Joints**: 7


---

## 1. Diagrama de Estructura


```

============================================================
DIAGRAMA DE ESTRUCTURA DEL ROBOT
============================================================

Base: world

     +---------+
     | world     |
     +---------+
  | ⚡ fixed    |
  +---------+
  | base_link |
  +---------+
    | ↻ revolute |
    +---------+
    | Link1     |
    +---------+
      | ↻ revolute |
      +---------+
      | Link2     |
      +---------+
        | ↻ revolute |
        +---------+
        | Link3     |
        +---------+
          | ↻ revolute |
          +---------+
          | Link4     |
          +---------+
            | ↻ revolute |
            +---------+
            | Link5     |
            +---------+
              | ↻ revolute |
              +---------+
              | Link6     |
              +---------+

Leyenda:
  ↻ = Revolute (rotación)
  ↔ = Prismatic (deslizamiento)
  ⚡ = Fixed (fijo)
  ○ = Continuous (continuo)

```


---

## 2. Tabla de Articulaciones (Joints)


| # | Nombre | Tipo | Parent | Child | Eje | Límite Inferior | Límite Superior | Esfuerzo Max | Velocidad Max |

|---|--------|------|--------|-------|-----|-----------------|-----------------|--------------|---------------|

| 1 | world_joint | fixed | world | base_link |  |  |  |  |  |

| 2 | joint1 | revolute | base_link | Link1 | 0 0 1 | -6.28 | 6.28 | 150.0 | 3.14 |

| 3 | joint2 | revolute | Link1 | Link2 | 0 0 1 | -6.28 | 6.28 | 150.0 | 3.14 |

| 4 | joint3 | revolute | Link2 | Link3 | 0 0 1 | -2.9 | 2.9 | 150.0 | 3.14 |

| 5 | joint4 | revolute | Link3 | Link4 | 0 0 1 | -6.28 | 6.28 | 50.0 | 3.14 |

| 6 | joint5 | revolute | Link4 | Link5 | 0 0 1 | -6.28 | 6.28 | 50.0 | 3.14 |

| 7 | joint6 | revolute | Link5 | Link6 | 0 0 1 | -6.28 | 6.28 | 50.0 | 3.14 |


### Descripción de tipos de joints:

- **revolute**: Rotación limitada entre dos ángulos

- **prismatic**: Movimiento lineal limitado

- **continuous**: Rotación ilimitada

- **fixed**: Sin movimiento (fijo)


---

## 3. Tabla de Eslabones (Links)


| # | Nombre | Masa (kg) | Geometría Visual | Geometría Colisión |

|---|--------|-----------|-------------------|--------------------|

| 1 | world | - | - | - |

| 2 | base_link | 5.0 | Mesh: base_link.STL | Mesh: base_link.STL |

| 3 | Link1 | 8.0 | Mesh: Link1.STL | Mesh: Link1.STL |

| 4 | Link2 | 6.0 | Mesh: Link2.STL | Mesh: Link2.STL |

| 5 | Link3 | 4.0 | Mesh: Link3.STL | Mesh: Link3.STL |

| 6 | Link4 | 3.0 | Mesh: Link4.STL | Mesh: Link4.STL |

| 7 | Link5 | 2.0 | Mesh: Link5.STL | Mesh: Link5.STL |

| 8 | Link6 | 1.0 | Mesh: Link6.STL | Mesh: Link6.STL |


---

## 4. Detalle de Cada Articulación


### world_joint


**Tipo**: `fixed`


**Conexión**:

- Padre (Parent): `world`

- Hijo (Child): `base_link`


**Origen**:

- Posición (xyz): `0 0 0.03`

- Rotación (rpy): `0 0 0`


**Límites de movimiento**:


---

### joint1


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `base_link`

- Hijo (Child): `Link1`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `0 0 0.23`

- Rotación (rpy): `0 0 0`


**Límites de movimiento**:

- Mínimo: -6.28 rad

- Máximo: 6.28 rad

- Esfuerzo máximo: 150.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 10.0

- Fricción: 0.5


---

### joint2


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `Link1`

- Hijo (Child): `Link2`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `1.5708 1.5708 0`


**Límites de movimiento**:

- Mínimo: -6.28 rad

- Máximo: 6.28 rad

- Esfuerzo máximo: 150.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 10.0

- Fricción: 0.5


---

### joint3


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `Link2`

- Hijo (Child): `Link3`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `-0.8252 0 0.0468`

- Rotación (rpy): `0 0 0`


**Límites de movimiento**:

- Mínimo: -2.9 rad

- Máximo: 2.9 rad

- Esfuerzo máximo: 150.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 8.0

- Fricción: 0.4


---

### joint4


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `Link3`

- Hijo (Child): `Link4`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `-0.746 0 0.1288`

- Rotación (rpy): `0 0 -1.5708`


**Límites de movimiento**:

- Mínimo: -6.28 rad

- Máximo: 6.28 rad

- Esfuerzo máximo: 50.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 5.0

- Fricción: 0.3


---

### joint5


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `Link4`

- Hijo (Child): `Link5`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `0 -0.1288 0`

- Rotación (rpy): `1.5708 0 0`


**Límites de movimiento**:

- Mínimo: -6.28 rad

- Máximo: 6.28 rad

- Esfuerzo máximo: 50.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 3.0

- Fricción: 0.2


---

### joint6


**Tipo**: `revolute`


**Conexión**:

- Padre (Parent): `Link5`

- Hijo (Child): `Link6`


**Eje de rotación**: `0 0 1`


**Origen**:

- Posición (xyz): `0 0.1365 0`

- Rotación (rpy): `-1.5708 0 0`


**Límites de movimiento**:

- Mínimo: -6.28 rad

- Máximo: 6.28 rad

- Esfuerzo máximo: 50.0 Nm

- Velocidad máxima: 3.14 rad/s


**Dinámica**:

- Amortiguación: 2.0

- Fricción: 0.1


---

## 5. Detalle de Cada Eslabón


### world


---

### base_link

**Masa**: 5.0 kg


**Tensor de Inercia**:

```

[0.01, 0.0, 0.0]

[0.0, 0.01, 0.0]

[0.0, 0.0, 0.01]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/base_link.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/base_link.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link1

**Masa**: 8.0 kg


**Tensor de Inercia**:

```

[0.05, 0.0, 0.0]

[0.0, 0.05, 0.0]

[0.0, 0.0, 0.02]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link1.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link1.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link2

**Masa**: 6.0 kg


**Tensor de Inercia**:

```

[0.1, 0.0, 0.0]

[0.0, 0.1, 0.0]

[0.0, 0.0, 0.02]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link2.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link2.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link3

**Masa**: 4.0 kg


**Tensor de Inercia**:

```

[0.05, 0.0, 0.0]

[0.0, 0.05, 0.0]

[0.0, 0.0, 0.01]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link3.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link3.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link4

**Masa**: 3.0 kg


**Tensor de Inercia**:

```

[0.01, 0.0, 0.0]

[0.0, 0.01, 0.0]

[0.0, 0.0, 0.005]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link4.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link4.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link5

**Masa**: 2.0 kg


**Tensor de Inercia**:

```

[0.005, 0.0, 0.0]

[0.0, 0.005, 0.0]

[0.0, 0.0, 0.003]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link5.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link5.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

### Link6

**Masa**: 1.0 kg


**Tensor de Inercia**:

```

[0.002, 0.0, 0.0]

[0.0, 0.002, 0.0]

[0.0, 0.0, 0.001]

```


**Geometría Visual**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link6.STL`

- Escala: ``


**Geometría de Colisión**:

- Tipo: `mesh`

- Archivo: `package://cra_description/meshes/cr20/Link6.STL`


**Origen**:

- Posición (xyz): `0 0 0`

- Rotación (rpy): `0 0 0`


---

## 6. Archivos de Mesh (Modelos 3D)


| Link | Ruta del Mesh |

|------|---------------|

| base_link | `package://cra_description/meshes/cr20/base_link.STL` |

| Link1 | `package://cra_description/meshes/cr20/Link1.STL` |

| Link2 | `package://cra_description/meshes/cr20/Link2.STL` |

| Link3 | `package://cra_description/meshes/cr20/Link3.STL` |

| Link4 | `package://cra_description/meshes/cr20/Link4.STL` |

| Link5 | `package://cra_description/meshes/cr20/Link5.STL` |

| Link6 | `package://cra_description/meshes/cr20/Link6.STL` |


---

## 7. Resumen de Límites de Movimiento


| Joint | Tipo | Rango de Movimiento | Velocidad Máx |

|-------|------|-------------------|---------------|

| world_joint | fixed | N/A (fijo) |  |

| joint1 | revolute | -6.28 a 6.28 rad | 3.14 |

| joint2 | revolute | -6.28 a 6.28 rad | 3.14 |

| joint3 | revolute | -2.9 a 2.9 rad | 3.14 |

| joint4 | revolute | -6.28 a 6.28 rad | 3.14 |

| joint5 | revolute | -6.28 a 6.28 rad | 3.14 |

| joint6 | revolute | -6.28 a 6.28 rad | 3.14 |


---

## 8. Ejemplos de Valores


### Posición Home (inicial)

```

joint1: 0 rad

joint2: -45° (-0.785 rad)

joint3: 60° (1.047 rad)

joint4: -90° (-1.571 rad)

joint5: -90° (-1.571 rad)

joint6: 0 rad

```

### Ejemplo de comando de trayectoria

```python

# Ejemplo de posiciones articulares (en radianes)

positions = [

    0.0,      # joint1

    -0.785,   # joint2 (-45°)

    1.047,   # joint3 (60°)

    -1.571,  # joint4 (-90°)

    -1.571,  # joint5 (-90°)

    0.0      # joint6

]

```


---

*Reporte generado automáticamente por analyze_urdf.py*
