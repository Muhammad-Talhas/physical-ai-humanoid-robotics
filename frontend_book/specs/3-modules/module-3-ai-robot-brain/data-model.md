# Module 3: The AI-Robot Brain (NVIDIA Isaac™) - Data Model

## Key Entities

### 1. Perception Pipeline
- **Attributes**:
  - pipeline_id: string (unique identifier)
  - pipeline_type: string (VSLAM, Object Detection, Semantic Segmentation, Depth Estimation)
  - input_sensors: array (camera, LiDAR, IMU, RGB-D, stereo)
  - processing_rate: float (Hz)
  - accuracy_metrics: object (precision, recall, mAP, F1_score)
  - gpu_resources: object (memory_usage, compute_units, power_consumption)
  - status: string (idle, processing, error, degraded)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime
  - version: semantic version string

### 2. Navigation Waypoint
- **Attributes**:
  - waypoint_id: string (unique identifier)
  - position: object (x, y, z coordinates in meters)
  - orientation: object (quaternion: x, y, z, w)
  - navigation_behavior: string (explore, avoid, follow, patrol, dock)
  - safety_constraints: object (min_distance, max_velocity, max_acceleration)
  - priority: integer (1-10, higher is more important)
  - accessibility: string (all_terrain, flat_ground, stairs, ramps)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime

### 3. Synthetic Data Sample
- **Attributes**:
  - sample_id: string (unique identifier)
  - domain: string (indoor, outdoor, warehouse, home, office)
  - sensor_type: string (camera, depth, LiDAR, IMU, thermal)
  - annotation_quality: string (perfect, noisy, partial, synthetic_only)
  - domain_randomization_params: object (lighting, textures, occlusions, weather)
  - source: string (simulation, real_world, synthetic)
  - label_accuracy: float (0.0-1.0 confidence score)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime
  - version: semantic version string

### 4. Isaac ROS Node
- **Attributes**:
  - node_name: string (unique identifier)
  - processing_type: string (perception, localization, mapping, navigation, control)
  - input_topics: array (ROS topic names)
  - output_topics: array (ROS topic names)
  - gpu_acceleration: boolean
  - performance_metrics: object (latency, throughput, accuracy, resource_usage)
  - status: string (running, paused, error, unconfigured)
  - gpu_memory_required: integer (MB)
  - processing_rate: float (Hz)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime

### 5. Navigation Map
- **Attributes**:
  - map_id: string (unique identifier)
  - name: string (descriptive name)
  - resolution: float (meters per pixel)
  - origin: object (x, y, z, roll, pitch, yaw in map frame)
  - size: object (width, height in meters)
  - occupancy_grid: binary (static map data)
  - costmap_layers: array (static, inflation, obstacle, voxel)
  - navigation_areas: array (object with area_id, type, boundaries)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime
  - map_source: string (slam, manual, semantic)

### 6. VSLAM State
- **Attributes**:
  - state_id: string (unique identifier)
  - robot_pose: object (position and orientation with covariance)
  - map_points: integer (number of landmarks in map)
  - tracking_quality: string (good, degraded, lost)
  - processing_time: float (ms per frame)
  - keyframes: integer (number of keyframes in map)
  - loop_closure_detected: boolean
  - last_loop_closure_time: ISO 8601 datetime
  - map_coverage: float (0.0-1.0 ratio of explored area)
  - created_timestamp: ISO 8601 datetime
  - modified_timestamp: ISO 8601 datetime

## Relationships

### Perception Pipeline connects to:
- Multiple Isaac ROS Nodes (processing chain)
- Input Sensors (data sources)
- Output Navigation Systems (action triggers)
- Synthetic Data Samples (training data source)

### Navigation Waypoint connects to:
- Navigation Map (spatial context)
- Perception Pipeline (obstacle detection)
- Robot State (current position)
- Navigation Path (waypoint sequence)

### Isaac ROS Node connects to:
- ROS Topics (input/output streams)
- GPU Resources (compute allocation)
- Other Isaac ROS Nodes (pipeline connections)
- Navigation System (control outputs)

### Navigation Map connects to:
- Navigation Waypoints (defined locations)
- Robot Poses (localization references)
- Costmap Layers (obstacle representation)
- Path Planning System (global/local planning)

## State Models

### Navigation State Machine
```
IDLE → LOCALIZING → NAVIGATING → AVOIDING_OBSTACLES → REACHED_GOAL
  ↓         ↓            ↓              ↓                  ↓
ERROR ←----┴------------┴--------------┴------------------┘
  ↓
RECOVERY → NAVIGATING
```
- Error states: FAILED_LOCALIZATION, OBSTACLE_BLOCKED, SAFETY_VIOLATION, LOST

### Perception State Machine
```
INITIALIZING → CALIBRATING → PROCESSING → ANALYZING → REPORTING
     ↓            ↓            ↓           ↓           ↓
   ERROR ←-------┴------------┴-----------┴-----------┘
     ↓
RECOVERY → PROCESSING
```
- Error states: SENSOR_FAILURE, GPU_UNAVAILABLE, DEGRADED_PERFORMANCE, INITIALIZATION_FAILED

### VSLAM State Machine
```
INITIALIZING → TRACKING → MAPPING → OPTIMIZING → LOCALIZING
     ↓           ↓          ↓         ↓          ↓
   ERROR ←------┴----------┴---------┴----------┘
     ↓
RECOVERY → TRACKING
```
- Error states: TRACKING_LOST, INSUFFICIENT_FEATURES, MAP_DIVERGENCE

## Validation Rules

### Perception Pipeline Validation
- processing_rate must be > 0 and ≤ 1000 Hz
- accuracy_metrics values must be between 0.0 and 1.0
- gpu_resources must not exceed available hardware limits
- input_sensors must be compatible with pipeline_type

### Navigation Waypoint Validation
- position coordinates must be within map boundaries
- orientation quaternion must be normalized
- priority must be between 1 and 10
- accessibility must match robot capabilities

### Isaac ROS Node Validation
- input_topics and output_topics must follow ROS naming conventions
- gpu_memory_required must be ≤ available GPU memory
- processing_type must be one of the defined values
- node_name must be unique within the system

### Navigation Map Validation
- resolution must be > 0 and ≤ 1.0 meters per pixel
- size dimensions must be positive
- occupancy grid values must be 0 (free), 100 (occupied), or -1 (unknown)
- origin must be within reasonable coordinate bounds

## Schema Requirements

### All entities must include:
- created_timestamp: ISO 8601 datetime format
- modified_timestamp: ISO 8601 datetime format
- unique identifier following the pattern: [entity_type]_#### (where #### is a 4-digit number)
- version: semantic version string (e.g., "1.0.0")

### Data Types
- string: UTF-8 encoded text
- integer: 32-bit signed integer
- float: 64-bit floating point number
- boolean: true/false values
- object: key-value pairs following JSON structure
- array: ordered list of values
- datetime: ISO 8601 format (YYYY-MM-DDTHH:MM:SS.sssZ)