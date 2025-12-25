# Unity HRI Scenario Example

## Human-Robot Interaction in Unity Environment

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      UNITY HRI SCENARIO                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐  │
│  │                    3D SCENE HIERARCHY                           │  │
│  │                                                                 │  │
│  │  Environment (Main)                                             │  │
│  │  ├─ Lighting Setup                                               │  │
│  │  │   ├─ Directional Light (Sun)                                  │  │
│  │  │   └─ Point Lights (Indoor)                                    │  │
│  │  ├─ Ground Plane                                                 │  │
│  │  ├─ Furniture & Obstacles                                        │  │
│  │  │   ├─ Tables, Chairs, Walls                                    │  │
│  │  │   └─ Interactive Objects                                      │  │
│  │  ├─ Human Avatar                                                 │  │
│  │  │   ├─ 3D Model with Animation Rig                              │  │
│  │  │   ├─ Motion Capture Integration                               │  │
│  │  │   └─ Gesture Recognition System                               │  │
│  │  └─ Robot Model                                                  │  │
│  │      ├─ URDF Import via Unity Robotics Package                   │  │
│  │      ├─ Physics Colliders                                        │  │
│  │      ├─ Sensor Visualizers                                       │  │
│  │      └─ Animation Controllers                                    │  │
│  └─────────────────────────────────────────────────────────────────┘  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## HRI Interaction Scenarios

### Scenario 1: Gesture-Based Control
```
┌─────────────────────────────────────────────────────────────────┐
│                GESTURE RECOGNITION FLOW                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Human Movement     Unity Processing        Robot Response     │
│      ↓                   ↓                      ↓              │
│  ┌─────────┐    ┌─────────────────────┐    ┌─────────────┐    │
│  │  GESTURE│───▶│ GESTURE RECOGNITION │───▶│  ROBOT      │    │
│  │  CAPTURE│    │ • Pose estimation   │    │  COMMAND    │    │
│  │ • Kinect│    │ • Gesture matching  │    │ • Move arm  │    │
│  │ • Camera│    │ • Confidence score  │    │ • Navigate  │    │
│  │ • IMU   │    │ • Temporal filtering│    │ • Grasp     │    │
│  └─────────┘    └─────────────────────┘    └─────────────┘    │
│                                                                 │
│  Example: Hand raise → Robot raises arm → LED feedback         │
└─────────────────────────────────────────────────────────────────┘
```

### Scenario 2: Voice Command Interface
```
┌─────────────────────────────────────────────────────────────────┐
│                 VOICE COMMAND FLOW                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Voice Input     Unity Processing        Robot Execution       │
│      ↓                   ↓                      ↓              │
│  ┌─────────┐    ┌─────────────────────┐    ┌─────────────┐    │
│  │ SPEECH  │───▶│ SPEECH RECOGNITION  │───▶│  NAVIGATION │    │
│  │ RECOGNITION│ │ • Audio processing  │    │  SYSTEM     │    │
│  │ • Microphone││ • Keyword spotting  │    │ • Path plan │    │
│  │ • Noise    │ │ • Intent parsing    │    │ • Motion    │    │
│  │   filtering│ │ • Confidence score  │    │   control   │    │
│  └─────────┘    └─────────────────────┘    └─────────────┘    │
│                                                                 │
│  Example: "Go to kitchen" → Navigate → Status update           │
└─────────────────────────────────────────────────────────────────┘
```

## Unity Scene Architecture for HRI

```
┌─────────────────────────────────────────────────────────────────┐
│                 UNITY HRI ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    CLIENT LAYER                           │  │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐ │  │
│  │  │   HUMAN INPUT   │  │        VISUALIZATION          │ │  │
│  │  │                 │  │                                 │ │  │
│  │  │ • VR Headset    │  │ • Robot state visualization     │ │  │
│  │  │ • Hand controllers││ • Sensor data overlay           │ │  │
│  │  │ • Eye tracking  │  │ • Path planning visualization   │ │  │
│  │  │ • Voice input   │  │ • Interaction feedback          │ │  │
│  │  └─────────────────┘  └─────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                │                                │
│                                ▼                                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                  UNITY LOGIC LAYER                        │  │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐ │  │
│  │  │   INTERACTION   │  │        ROS BRIDGE             │ │  │
│  │  │   SYSTEM        │  │                                 │ │  │
│  │  │                 │  │ • Unity Robotics Package        │ │  │
│  │  │ • Gesture       │  │ • Message serialization         │ │  │
│  │  │   recognition   │  │ • Topic publishing/subscribing  │ │  │
│  │  │ • Voice         │  │ • Service calls                 │ │  │
│  │  │   processing    │  │ • TF transformations            │ │  │
│  │  │ • State         │  │ • Action clients                │ │  │
│  │  │   management    │  │                                 │ │  │
│  │  └─────────────────┘  └─────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                │                                │
│                                ▼                                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                 ROS/ROS2 LAYER                            │  │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐ │  │
│  │  │  ROBOT CONTROL  │  │       PERCEPTION              │ │  │
│  │  │                 │  │                                 │ │  │
│  │  │ • Navigation    │  │ • SLAM                          │ │  │
│  │  │ • Manipulation  │  │ • Object detection              │ │  │
│  │  │ • Path planning │  │ • Human tracking                │ │  │
│  │  │ • Motion        │  │ • Scene understanding           │ │  │
│  │  │   control       │  │ • Gesture recognition (robot)   │ │  │
│  │  └─────────────────┘  └─────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Example HRI Scene Implementation

### 1. Human Avatar Controller Script
```csharp
// HumanAvatarController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class HumanAvatarController : MonoBehaviour
{
    [SerializeField] private float interactionDistance = 2.0f;
    [SerializeField] private Camera mainCamera;

    private ROSConnection ros;
    private NavMeshAgent robotAgent;

    void Start()
    {
        ros = ROSConnection.instance;
        // Subscribe to robot status topics
        ros.Subscribe<Nav_msgs.OdometryMsg>(
            "/robot/odom", RobotStatusCallback);
    }

    void Update()
    {
        HandleHumanInput();
        CheckProximityInteractions();
    }

    void HandleHumanInput()
    {
        // Process keyboard/mouse input for avatar movement
        float moveX = Input.GetAxis("Horizontal");
        float moveZ = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(moveX, 0, moveZ) * Time.deltaTime * 3.0f;
        transform.Translate(movement);
    }

    void CheckProximityInteractions()
    {
        // Detect nearby robots for interaction
        Collider[] nearby = Physics.OverlapSphere(
            transform.position, interactionDistance);

        foreach (Collider col in nearby)
        {
            if (col.CompareTag("Robot"))
            {
                // Trigger interaction UI or behavior
                HighlightInteraction(col.transform);
            }
        }
    }

    void RobotStatusCallback(Nav_msgs.OdometryMsg robotOdom)
    {
        // Update robot visualization based on ROS data
        Vector3 robotPos = new Vector3(
            (float)robotOdom.pose.pose.position.x,
            (float)robotOdom.pose.pose.position.y,
            (float)robotOdom.pose.pose.position.z);

        // Update robot game object position
        GameObject.Find("RobotModel").transform.position = robotPos;
    }
}
```

### 2. Robot Visualization Manager
```csharp
// RobotVisualizationManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotVisualizationManager : MonoBehaviour
{
    [SerializeField] private GameObject robotModel;
    [SerializeField] private Material[] sensorMaterials;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to various sensor topics
        ros.Subscribe<Sensor_msgs.LaserScanMsg>(
            "/robot/laser_scan", UpdateLidarVisualization);
        ros.Subscribe<Sensor_msgs.ImageMsg>(
            "/robot/camera/image_raw", UpdateCameraVisualization);
        ros.Subscribe<Sensor_msgs.ImuMsg>(
            "/robot/imu/data", UpdateImuVisualization);
    }

    void UpdateLidarVisualization(Sensor_msgs.LaserScanMsg scan)
    {
        // Create visual representation of LiDAR data
        for (int i = 0; i < scan.ranges.Count; i++)
        {
            float angle = scan.angle_min + i * scan.angle_increment;
            float distance = (float)scan.ranges[i];

            if (distance < scan.range_max && distance > scan.range_min)
            {
                Vector3 point = new Vector3(
                    distance * Mathf.Cos(angle),
                    0,
                    distance * Mathf.Sin(angle));

                // Visualize point in Unity scene
                CreateLidarPoint(point);
            }
        }
    }

    void CreateLidarPoint(Vector3 position)
    {
        GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        point.transform.position = position + robotModel.transform.position;
        point.transform.localScale = Vector3.one * 0.05f;
        point.GetComponent<Renderer>().material = sensorMaterials[0];

        // Destroy after a few seconds to avoid clutter
        Destroy(point, 2.0f);
    }

    void UpdateCameraVisualization(Sensor_msgs.ImageMsg image)
    {
        // Update camera feed texture in Unity UI
        // (Implementation would involve texture conversion)
    }

    void UpdateImuVisualization(Sensor_msgs.ImuMsg imu)
    {
        // Update robot orientation based on IMU data
        robotModel.transform.rotation = new Quaternion(
            (float)imu.orientation.x,
            (float)imu.orientation.y,
            (float)imu.orientation.z,
            (float)imu.orientation.w);
    }
}
```

## HRI User Interface Components

```
┌─────────────────────────────────────────────────────────────────┐
│                   HRI UI COMPONENTS                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    OVERLAY UI                             │  │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐ │  │
│  │  │  INTERACTION    │  │        STATUS DISPLAY         │ │  │
│  │  │  INDICATORS     │  │                                 │ │  │
│  │  │                 │  │ • Robot battery level           │ │  │
│  │  │ • Proximity     │  │ • Task progress                 │ │  │
│  │  │   indicators    │  │ • Sensor status                 │ │  │
│  │  │ • Gesture       │  │ • Connection status             │ │  │
│  │  │   recognition   │  │ • Emergency stop                │ │  │
│  │  │ • Voice         │  │ • Performance metrics           │ │  │
│  │  │   feedback      │  │                                 │ │  │
│  │  └─────────────────┘  └─────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                 VR CONTROLLER UI                          │  │
│  │  ┌─────────────────┐  ┌─────────────────────────────────┐ │  │
│  │  │  CONTROLLER     │  │       GESTURE MENU            │ │  │
│  │  │  INTERFACE      │  │                                 │ │  │
│  │  │                 │  │ • Point: Navigate robot         │ │  │
│  │  │ • Button        │  │ • Grab: Manipulate object       │ │  │
│  │  │   mapping       │  │ • Wave: Request attention       │ │  │
│  │  │ • Haptic        │  │ • Thumbs up: Confirm action     │ │  │
│  │  │   feedback      │  │ • Peace sign: Take photo        │ │  │
│  │  │ • Menu          │  │                                 │ │  │
│  │  │   navigation    │  │                                 │ │  │
│  │  └─────────────────┘  └─────────────────────────────────┘ │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Safety and Validation Framework

```
┌─────────────────────────────────────────────────────────────────┐
│                   HRI SAFETY FRAMEWORK                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   SAFETY        │    │   MONITORING    │    │   RESPONSE  │ │
│  │   BOUNDARIES    │    │   SYSTEM        │    │   SYSTEM    │ │
│  │                 │    │                 │    │             │ │
│  │ • Collision     │    │ • Distance      │    │ • Emergency │ │
│  │   detection     │    │   monitoring    │    │   stop      │ │
│  │ • Safe zones    │    │ • Interaction   │    │ • Path      │ │
│  │ • Speed limits  │    │   tracking      │    │   replanning│ │
│  │ • Force limits  │    │ • Gesture       │    │ • Alert     │ │
│  │ • Timeouts      │    │   validation    │    │   system    │ │
│  └─────────────────┘    └─────────────────┘    └─────────────┘ │
│                                │                      │        │
│                                ▼                      ▼        │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    SAFETY LOGIC                           │  │
│  │ • Continuous safety assessment during interaction       │  │
│  │ • Predictive safety analysis using ML models            │  │
│  │ • Human-in-the-loop safety override capabilities        │  │
│  │ • Compliance with ISO/TS 15066 for HRI safety           │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Performance Metrics Dashboard

```
┌─────────────────────────────────────────────────────────────────┐
│                   HRI PERFORMANCE DASHBOARD                     │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  INTERACTION    │  │  RESPONSE       │  │  SATISFACTION   │ │
│  │  SUCCESS RATE   │  │  TIME           │  │  SCORE          │ │
│  │                 │  │                 │  │                 │ │
│  │  ████████░░ 80% │  │  Avg: 0.8s      │  │  ⭐⭐⭐⭐░ 4.0/5.0 │ │
│  │                 │  │  Min: 0.2s      │  │                 │ │
│  │  Success/Failure│  │  Max: 2.1s      │  │  Survey-based  │ │
│  │  metrics       │  │  95% < 1.5s     │  │  evaluation    │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                   DETAILED METRICS                        │  │
│  │ • Task completion time: 2.3 min average                   │  │
│  │ • Error rate: 2.1%                                        │  │
│  │ • User engagement: 87% active participation               │  │
│  │ • System uptime: 99.2%                                    │  │
│  │ • Communication latency: <50ms 95% of the time           │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---
*Figure 6: Unity-based Human-Robot Interaction scenario showing architecture, implementation, and validation framework*