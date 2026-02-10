# Unity Jacobian IK Solver for Robot Arms

A flexible, high-performance Jacobian-based Inverse Kinematics (IK) solver for robot manipulators in Unity. Originally developed for the Franka Emika Panda robot arm, this solver has proven adaptable to other robot platforms including UR3e.

## 🌟 Features

- **Jacobian-based IK**: Fast iterative solver using SVD pseudoinverse
- **Unity ArticulationBody Integration**: Seamlessly works with Unity's physics engine
- **Flexible Architecture**: Easily adaptable to different robot configurations
- **Joint Limit Handling**: Respects position and velocity constraints
- **Workspace Validation**: Built-in collision and reachability checks
- **End Effector Tools**: Support for grippers and tool offsets

## 📦 What's Included

### Core IK Components

- `Chain.cs` - Kinematic chain representation and forward kinematics
- `Segment.cs` - Individual joint/link representation
- `Jacobian.cs` - Jacobian matrix computation and operations
- `FastIterSolve.cs` - Iterative IK solver with convergence detection
- `IKJoint.cs` - Joint configuration and constraints

### Robot Interface

- `PandaRobot.cs` - Complete Panda robot controller (reference implementation)
- `PandaControlDemo.cs` - Example control scripts
- `PandaControlIKKeyboard.cs` - Keyboard teleoperation example

## 🚀 Quick Start

### 1. Setup Your Robot in Unity

```csharp
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

public class MyRobotController : MonoBehaviour
{
    private PandaRobot robot;
    
    void Start()
    {
        // Initialize robot from GameObject with ArticulationBody hierarchy
        robot = new PandaRobot(gameObject);
    }
    
    void FixedUpdate()
    {
        // Define target pose
        Matrix4x4 targetPose = Matrix4x4.TRS(
            new Vector3(0.5f, 0.3f, 0.2f),  // position
            Quaternion.Euler(0, 90, 0),      // rotation
            Vector3.one
        );
        
        // Solve IK
        Vector&lt;float&gt; jointAngles = robot.SolveInverseKinematics(
            targetPose, 
            robot.q_initial  // initial guess
        );
        
        // Drive joints to solution
        robot.DriveJoints(jointAngles);
    }
}
```

### 2. Requirements

- **Unity 2020.3+** (tested up to 2022.3)
- **Math.NET Numerics** library
- Robot model with properly configured ArticulationBody chain

## 🔧 Adapting to Your Robot

This IK solver has been successfully adapted from Panda (7-DOF) to UR3e (6-DOF), demonstrating its flexibility. The adaptation process involves:

### Key Adaptation Points

1. **Joint Limits** - Update min/max angles for your robot
2. **Joint Axis Directions** - Configure rotation axes in `Segment.cs`
3. **DH Parameters** - Ensure your URDF/model matches your robot
4. **Workspace Constraints** - Define reachability limits
5. **Tool Center Point (TCP)** - Calculate correct end effector offset

For detailed adaptation instructions, see **[ADAPTATION_GUIDE.md](ADAPTATION_GUIDE.md)**

## 📖 Example: Panda Robot Specifications

```csharp
// Joint limits (radians)
float[] maxJoint = { 2.8973f, 1.7628f, 2.8973f, 3.0718f, 2.8973f, 0.0175f, 2.8973f };
float[] minJoint = { -2.8973f, -1.7628f, -2.8973f, 0.0698f, -2.8973f, -3.7525f, -2.8973f };

// Workspace
float lateral_radius = 855mm;
float max_height = 1190mm;
float min_height = -360mm;

// Velocity limit
float max_joint_velocity = 40.0°/s;
```

## 🎯 Successful Adaptations

| Robot | DOF | Status | Notes |
|-------|-----|--------|-------|
| **Franka Emika Panda** | 7 | ✅ Original | Full dynamics support |
| **Universal Robots UR3e** | 6 | ✅ Adapted | Production-ready |
| **Your Robot** | ? | 🚀 Ready | Use the guide below! |

## 🤖 AI-Assisted Adaptation

We provide a detailed prompt for AI agents (like GPT-4, Claude, etc.) to help you adapt this IK solver to your robot. This has been successfully tested with UR3e adaptation.

See **[ADAPTATION_GUIDE.md](ADAPTATION_GUIDE.md)** for the complete AI adaptation prompt and step-by-step instructions.

## 📐 Architecture Overview

```
GameObject (Robot Root)
├── ArticulationBody (Base) [Fixed]
├── ArticulationBody (Joint1) [Revolute]
├── ArticulationBody (Joint2) [Revolute]
├── ...
├── ArticulationBody (JointN) [Revolute]
└── GameObject (end_effector marker)
    └── GameObject (TCP)
```

The solver:
1. Traverses the ArticulationBody hierarchy
2. Builds kinematic chain from joint configurations
3. Computes Jacobian matrix from current joint state
4. Iteratively solves IK using damped least-squares
5. Respects joint limits and singularity handling

## 🛠️ Advanced Features

### Custom Weights

```csharp
// Prioritize position over orientation
float[] weights = { 1.0f, 1.0f, 1.0f, 0.1f, 0.1f, 0.1f };
Vector&lt;float&gt; solution = robot.SolveInverseKinematics(targetPose, initialGuess, weights);
```

### Velocity-Limited Motion

```csharp
// Smooth incremental movement
robot.DriveJointsIncremental(goalJoints, robot.q_initial);
```

### Workspace Validation

```csharp
if (robot.InsidePandaWorkspace(targetPose))
{
    // Safe to solve IK
}
```

## 📚 API Reference

### PandaRobot Class

**Constructor**
- `PandaRobot(GameObject robotBase)` - Initialize from Unity GameObject

**IK Methods**
- `SolveInverseKinematics(Matrix4x4 goalPose, Vector&lt;float&gt; q_init, params float[] weights)` - Solve IK
- `InsidePandaWorkspace(Matrix4x4 pose)` - Check workspace limits
- `InsideVelocityLimits(Matrix4x4 pose, float dt)` - Check velocity feasibility

**Control Methods**
- `DriveJoints(Vector&lt;float&gt; q_goal)` - Direct joint control
- `DriveJointsIncremental(Vector&lt;float&gt; q_start, Vector&lt;float&gt; q_goal)` - Smooth motion

### Chain Class

- `UpdateChainKinematics(Vector&lt;float&gt; q_state)` - Update FK
- `JntToCart()` - Compute forward kinematics
- `BuildJacobian(Vector&lt;float&gt; q_state)` - Compute Jacobian matrix

## 🐛 Troubleshooting

### IK Not Converging
- Check joint limits are correct
- Verify target is within workspace
- Try different initial guess
- Increase `maxCycles` limit

### Incorrect End Effector Position
- Verify TCP offset in `GripperTool`
- Check URDF import scaling
- Validate DH parameters

### Unstable Motion
- Reduce velocity limits
- Check collision settings
- Tune ArticulationBody stiffness/damping

## 🤝 Contributing

This is a reference implementation. Feel free to:
- Adapt for your robot
- Improve the algorithms
- Add new features
- Share your adaptations

## 🙏 Acknowledgments

**This project is based on the excellent work by [jiafei1224](https://github.com/jiafei1224) in the [AR2-D2 project](https://github.com/jiafei1224/AR2-D2).**

Special thanks to the original author for creating this robust and extensible IK solver framework. The core algorithms and architecture from AR2-D2 have made it possible to successfully adapt this solver to multiple robot platforms.

### Original Project
- **Repository**: [AR2-D2](https://github.com/jiafei1224/AR2-D2)
- **Author**: [jiafei1224](https://github.com/jiafei1224)
- **Original Target**: Franka Emika Panda robot arm

### This Repository
- Successfully adapted for UR3e robot
- Added comprehensive documentation for community adaptation
- Enhanced with automatic joint axis detection
- Improved tool center point (TCP) offset calculation

### Additional Credits
- Built with Unity ArticulationBody system
- Math.NET Numerics for linear algebra
- Jacobian-based IK algorithm implementation

## 📄 License

[Add your license here - Note: Check original AR2-D2 project license and comply accordingly]

## 📞 Support

- Create an issue for bugs
- Check existing issues for solutions
- See ADAPTATION_GUIDE.md for robot-specific help
- Review the original [AR2-D2 project](https://github.com/jiafei1224/AR2-D2) for additional context

---

**Ready to adapt this to your robot?** Check out **[ADAPTATION_GUIDE.md](ADAPTATION_GUIDE.md)** for detailed instructions and an AI-ready prompt!
