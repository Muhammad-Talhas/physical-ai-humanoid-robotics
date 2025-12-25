# Gazebo World File Structure Diagram

## SDF (Simulation Description Format) Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    SDF WORLD FILE                               │
├─────────────────────────────────────────────────────────────────┤
│  <?xml version="1.0" ?>                                         │
│  <sdf version="1.7">                                            │
│    <world name="my_world">                                      │
│      <!-- Environment Settings -->                              │
│      <gravity>0 0 -9.8</gravity>                               │
│      <magnetic_field>0 0 0</magnetic_field>                    │
│      <atmosphere type="adiabatic">                              │
│        ...                                                      │
│      </atmosphere>                                              │
│                                                                 │
│      <!-- Physics Engine Configuration -->                      │
│      <physics name="default_physics" type="ode">                │
│        <max_step_size>0.001</max_step_size>                     │
│        <real_time_factor>1</real_time_factor>                   │
│        ...                                                      │
│      </physics>                                                 │
│                                                                 │
│      <!-- Models -->                                            │
│      <model name="robot_model">                                 │
│        ...                                                      │
│      </model>                                                   │
│      <model name="environment_object">                          │
│        ...                                                      │
│      </model>                                                   │
│                                                                 │
│      <!-- Lights -->                                            │
│      <light name="sun" type="directional">                      │
│        <pose>0 0 10 0 0 0</pose>                               │
│        ...                                                      │
│      </light>                                                   │
│                                                                 │
│      <!-- Plugins -->                                          │
│      <plugin name="my_plugin" filename="libmy_plugin.so">       │
│        ...                                                      │
│      </plugin>                                                  │
│    </world>                                                     │
│  </sdf>                                                         │
└─────────────────────────────────────────────────────────────────┘
```

## Hierarchical Structure

```
SDF ROOT
├── sdf (version attribute)
    └── world (name attribute)
        ├── gravity (x, y, z)
        ├── magnetic_field (x, y, z)
        ├── atmosphere (type attribute)
        │   ├── temperature
        │   ├── pressure
        │   └── ... other atmospheric properties
        ├── physics (name, type attributes)
        │   ├── max_step_size
        │   ├── real_time_factor
        │   ├── real_time_update_rate
        │   ├── solver
        │   │   ├── type (ode, bullet, dart)
        │   │   ├── iters
        │   │   └── ... solver parameters
        │   └── ... other physics parameters
        ├── light (name, type attributes)
        │   ├── pose
        │   ├── diffuse
        │   ├── specular
        │   ├── attenuation
        │   └── ... other light properties
        ├── model (name attribute)
        │   ├── static (true/false)
        │   ├── pose
        │   ├── link (collision, visual, inertial elements)
        │   ├── joint (parent, child, axis, limit)
        │   ├── plugin
        │   └── ... model-specific elements
        ├── include (for referencing other models)
        │   ├── uri (location of model)
        │   └── name (optional override)
        ├── state (runtime state information)
        └── plugin (world-level plugins)
            ├── name
            ├── filename
            └── custom parameters
```

## Example World File Structure

```
┌─────────────────────────────────────────────────────────────────┐
│              EXAMPLE WORLD FILE STRUCTURE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  my_world.world                                                 │
│  ├── Environment Setup                                          │
│  │   ├── Gravity: 0 0 -9.8 m/s²                                │
│  │   ├── Physics: ODE engine, dt=0.001s                        │
│  │   └── Atmosphere: adiabatic                                 │
│  ├── Lighting                                                   │
│  │   ├── Sun: directional light at (0,0,10)                    │
│  │   └── Ambient light: soft global illumination               │
│  ├── Terrain & Objects                                          │
│  │   ├── Ground plane with texture                             │
│  │   ├── Buildings/models from model database                  │
│  │   └── Obstacles with collision properties                   │
│  ├── Robots                                                     │
│  │   ├── Robot1: differential drive                            │
│  │   ├── Robot2: manipulator arm                               │
│  │   └── Sensors: LiDAR, cameras, IMU                          │
│  └── Plugins                                                    │
│      ├── ROS bridge plugin                                      │
│      ├── GUI plugins                                            │
│      └── Custom controllers                                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## World File Loading Process

```
┌─────────────────────────────────────────────────────────────────┐
│                LOADING SEQUENCE                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  START ──► Parse SDF ──► Validate Structure ──► Load Elements   │
│                   │                                            │
│                   ▼                                            │
│        Initialize Physics Engine ──► Set Environment          │
│                   │                                            │
│                   ▼                                            │
│        Load Models ──► Create Scene Graph ──► Initialize       │
│                   │                │        Controllers       │
│                   │                │                          │
│                   ▼                ▼                          │
│        Setup Sensors ──► Connect Plugins ──► Ready State       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Key Components Breakdown

### 1. Environment Settings
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   GRAVITY       │    │ MAGNETIC FIELD  │    │  ATMOSPHERE     │
│                 │    │                 │    │                 │
│  <gravity>      │    │ <magnetic_field>│    │ <atmosphere     │
│    0 0 -9.8     │    │   0 0 0         │    │   type="...">   │
│  </gravity>     │    │ </magnetic_field>│   │   ...           │
│                 │    │                 │    │ </atmosphere>   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 2. Physics Configuration
```
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICS ENGINE                               │
├─────────────────────────────────────────────────────────────────┤
│  <physics name="default_physics" type="ode">                    │
│    <!-- Time Stepping -->                                       │
│    <max_step_size>0.001</max_step_size>                         │
│    <real_time_factor>1.0</real_time_factor>                     │
│    <real_time_update_rate>1000</real_time_update_rate>          │
│                                                                 │
│    <!-- Solver Settings -->                                     │
│    <solver>                                                     │
│      <type>ode</type>                                           │
│      <iters>1000</iters>                                        │
│      <sor>1.3</sor>                                             │
│    </solver>                                                    │
│                                                                 │
│    <!-- Constraints -->                                         │
│    <constraints>                                                │
│      <cfm>0.0</cfm>                                             │
│      <erp>0.2</erp>                                             │
│      <contact_surface_layer>0.001</contact_surface_layer>      │
│      <contact_max_correcting_vel>100</contact_max_correcting_vel>│
│    </constraints>                                               │
│  </physics>                                                     │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Model Integration
```
┌─────────────────────────────────────────────────────────────────┐
│                      MODEL STRUCTURE                            │
├─────────────────────────────────────────────────────────────────┤
│  <model name="my_robot" canonical_link="base_link">            │
│    <!-- Static or Dynamic -->                                   │
│    <static>false</static>                                       │
│                                                                 │
│    <!-- Pose in World Frame -->                                 │
│    <pose>0 0 0.5 0 0 0</pose>                                  │
│                                                                 │
│    <!-- Links with Collisions and Visuals -->                   │
│    <link name="base_link">                                      │
│      <collision name="collision">...</collision>                │
│      <visual name="visual">...</visual>                         │
│      <inertial>...</inertial>                                   │
│    </link>                                                      │
│                                                                 │
│    <!-- Joints Connecting Links -->                             │
│    <joint name="joint1" type="revolute">                        │
│      <parent>base_link</parent>                                 │
│      <child>link1</child>                                       │
│      <axis>...</axis>                                           │
│    </joint>                                                     │
│                                                                 │
│    <!-- Model-level Plugins -->                                 │
│    <plugin name="controller" filename="libcontroller.so">       │
│      ...                                                        │
│    </plugin>                                                    │
│  </model>                                                       │
└─────────────────────────────────────────────────────────────────┘
```

## Best Practices

1. **Organization**: Group related objects spatially in the file
2. **Naming**: Use consistent, descriptive names for all elements
3. **Modularity**: Use `<include>` tags to reference standardized models
4. **Validation**: Always validate SDF files before loading
5. **Documentation**: Comment complex sections for maintainability

---
*Figure 4: Detailed structure of Gazebo world files using SDF format*