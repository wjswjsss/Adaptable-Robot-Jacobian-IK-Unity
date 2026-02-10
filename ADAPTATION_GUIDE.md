# Robot Adaptation Guide

This guide explains how to adapt the Panda Jacobian IK solver to your own robot arm. We've successfully used this process to adapt the solver from Panda (7-DOF) to UR3e (6-DOF).

## 📋 Table of Contents

1. [Overview](#overview)
2. [What Needs to Change](#what-needs-to-change)
3. [Step-by-Step Adaptation](#step-by-step-adaptation)
4. [AI-Assisted Adaptation Prompt](#ai-assisted-adaptation-prompt)
5. [Verification](#verification)

---

## Overview

The IK solver is designed to be robot-agnostic at its core. The adaptation primarily involves:
- Configuring robot-specific parameters (joint limits, DH parameters)
- Computing the correct joint axes for your robot
- Calculating proper tool center point (TCP) offsets
- Defining workspace boundaries


---

## What Needs to Change

Based on our successful Panda → UR3e adaptation, here are the key components to modify:

### 1. **Robot Class** (e.g., `PandaRobot.cs` → `YourRobot.cs`)

**Purpose**: High-level interface for your specific robot

**Changes Required**:
- Joint limits (min/max angles in radians)
- Default home position
- Workspace constraints (reach, height limits)
- Mass/inertia data (optional, for dynamics)

**Example Comparison**:

```csharp
// Panda (7 DOF)
float[] maxJoint = { 2.8973f, 1.7628f, 2.8973f, 3.0718f, 2.8973f, 0.0175f, 2.8973f };
float[] minJoint = { -2.8973f, -1.7628f, -2.8973f, 0.0698f, -2.8973f, -3.7525f, -2.8973f };

// UR3e (6 DOF)
float[] maxJoint = { 2π, 2π, 2π, 2π, 2π, 2π };
float[] minJoint = { -2π, -2π, -2π, -2π, -2π, -2π };
```

### 2. **Segment.cs - Joint Axis Calculation**

**Purpose**: Determines rotation axis for each joint

**Critical Change**: The Panda version uses **hardcoded** axes, while the adapted version **computes** them from Unity's `anchorRotation`.

**Panda Version** (Hardcoded - Robot Specific):
```csharp
// Segment.cs lines 121-128
if (this.index == 1) { this.jointIndex = new Vector3(0, 0, 1); }
else if (this.index == 2) { this.jointIndex = new Vector3(0, 0, -1); }
else if (this.index == 3) { this.jointIndex = new Vector3(0, 1, 0); }
else if (this.index == 4) { this.jointIndex = new Vector3(0, 1, 0); }
// ... hardcoded for each joint
```

**UR3e Version** (Computed - Robot Agnostic):
```csharp
// Segment.cs lines 110-123
// Unity ArticulationBody revolute joint defaults to rotating around local X axis
// Actual rotation axis = anchorRotation * Vector3.right
Vector3 computedAxis = newBody.anchorRotation * Vector3.right;

// Round to nearest principal axis to avoid floating point errors
this.jointIndex = new Vector3(
    Mathf.Abs(computedAxis.x) &gt; 0.5f ? Mathf.Sign(computedAxis.x) : 0f,
    Mathf.Abs(computedAxis.y) &gt; 0.5f ? Mathf.Sign(computedAxis.y) : 0f,
    Mathf.Abs(computedAxis.z) &gt; 0.5f ? Mathf.Sign(computedAxis.z) : 0f
);
```

**Why This Matters**: The computed version automatically adapts to any robot's joint configuration as long as the URDF import is correct.

### 3. **Chain.cs - End Effector Detection**

**Purpose**: Identifies where the kinematic chain ends and handles TCP offset

**Key Improvements in UR3e Version**:

**A. Flexible End Effector Detection**:
```csharp
// Panda (single name check)
if (cBody.name.Contains("end_effector")) { ... }

// UR3e (multiple name patterns)
private static readonly string[] DefaultEEStopNames = 
    { "end_effector", "TCP", "tcp", "HandE", "hand", "ee_link", "tool0" };
```

**B. Automatic TCP Offset Calculation**:
```csharp
// UR3e - Chain.cs lines 124-169
private GripperTool CreateGripperWithCorrectOffset()
{
    // Find last segment
    Segment lastSegment = this.segments[this.segments.Count - 1];
    
    // Find TCP transform (recursive search)
    Transform tcpTransform = FindTCPTransform(this.eeBody.transform);
    
    // Calculate world offset: TCP - LastSegment
    Vector3 worldOffset = tcpTransform.position - lastSegmentTransform.position;
    
    // Convert to last segment's local frame
    gripper.toolVector = Quaternion.Inverse(lastSegmentTransform.rotation) * worldOffset;
    
    return gripper;
}
```

**C. Improved Tool Vector Application**:
```csharp
// Panda applies toolVector during FK iteration
if (i == this.numberSegments - 1) {
    iterTransform = FwKinSegment(iterTransform, this.segments[i], this.Gripper.toolVector);
}

// UR3e applies toolVector AFTER FK completes (more robust)
// Chain.cs lines 430-446
if (this.Gripper != null &amp;&amp; this.Gripper.toolVector != Vector3.zero) {
    Quaternion endRotation = iterTransform.rotation;
    Vector3 toolOffsetWorld = endRotation * this.Gripper.toolVector;
    Matrix4x4 toolTransform = Matrix4x4.Translate(toolOffsetWorld);
    iterTransform = toolTransform * iterTransform;
}
```

### 4. **Chain.cs - Jacobian Calculation**

**Purpose**: Computes the Jacobian matrix for IK solving

**Critical Change**:
```csharp
// Panda 
Vector3 Jv = Vector3.Cross(cartDelta, segAxis);

// UR3e (regular cross product order)
Vector3 Jv = Vector3.Cross(segAxis, cartDelta);  // axis × r (correct for revolute joints)
```

**Additionally, UR3e version**:
- Uses computed joint axes from `Segment` (lines 552-561)
- Normalizes axes to ensure unit vectors
- More robust to different robot configurations

### 5. **Constructor Parameters**

**UR3e improvements**:
```csharp
// Chain.cs - More flexible constructor
public Chain(GameObject rootObject, Vector&lt;float&gt; jointMin, Vector&lt;float&gt; jointMax, 
             float jointVelMax = 20.0f, string[] customEENames = null)
```

Allows custom end effector detection patterns without code changes.

---

## Step-by-Step Adaptation

### Phase 1: Preparation

1. **Get Your Robot Model**
   - URDF file for your robot
   - Import into Unity using URDF Importer
   - Verify ArticulationBody chain is correct

2. **Gather Robot Specifications**
   - Joint limits (min/max angles)
   - DH parameters
   - Workspace envelope
   - Max joint velocities
   - TCP offset from last joint

### Phase 2: Code Adaptation

1. **Copy the Panda Implementation**
   ```bash
   cp PandaRobot.cs YourRobot.cs
   # Also copy all core files:
   # Chain.cs, Segment.cs, Jacobian.cs, FastIterSolve.cs, IKJoint.cs
   ```

2. **Update Segment.cs**
   - Replace hardcoded joint axes with computed axes (lines 108-130)
   - This is the **most critical change**

3. **Update Chain.cs**
   - Add flexible end effector detection (lines 53-87)
   - Implement `CreateGripperWithCorrectOffset()` (lines 124-169)
   - Fix Jacobian cross product order (line 563)
   - Update tool vector application (lines 430-446)

4. **Create YourRobot.cs**
   - Update joint limits
   - Set correct home position
   - Define workspace constraints
   - Remove Panda-specific constants

### Phase 3: Configuration

1. **Verify Joint Axes**
   - Add debug logging in `Segment.AddJoint()`
   - Check that computed axes match expected rotation axes
   - Test with simple joint rotations

2. **Calibrate TCP Offset**
   - Ensure TCP is correctly defined in your model
   - Verify `toolVector` calculation in debug logs
   - Test FK accuracy: move to known pose, check TCP position

3. **Tune Solver Parameters**
   - Adjust `maxCycles` if needed
   - Test convergence with various target poses
   - Verify joint limit enforcement

### Phase 4: Testing

1. **Forward Kinematics Test**
   ```csharp
   robot.DriveJoints(knownJointAngles);
   Vector3 resultingTCP = robot.Articulation.base2EETransform.GetPosition();
   // Compare with expected TCP position
   ```

2. **Inverse Kinematics Test**
   ```csharp
   Matrix4x4 targetPose = CreateTestPose();
   Vector&lt;float&gt; solution = robot.SolveInverseKinematics(targetPose, initialGuess);
   // Verify convergence and accuracy
   ```

3. **Workspace Test**
   - Test extreme positions
   - Verify joint limits are respected
   - Check singularity handling

---

## AI-Assisted Adaptation Prompt

Copy the following prompt and provide it to an AI assistant (Claude, GPT-4, etc.) along with your robot's specifications:

---

### 🤖 PROMPT START

I need to adapt a Jacobian-based IK solver from the Franka Emika Panda robot to **[YOUR ROBOT NAME]**. The original implementation is designed for Unity's ArticulationBody system.

**My Robot Specifications**:
- **Robot Name**: [e.g., UR5, KUKA iiwa, ABB IRB, etc.]
- **DOF**: [number of joints]
- **Joint Limits** (in radians or degrees, specify units):
  ```
  Joint 1: [min] to [max]
  Joint 2: [min] to [max]
  ...
  ```
- **Default Home Position**: [joint angles]
- **Workspace**:
  - Max reach: [mm/m]
  - Min reach: [mm/m]
  - Max height: [mm/m]
  - Min height: [mm/m]
- **Max Joint Velocities**: [deg/s or rad/s]
- **TCP Offset**: [describe or provide coordinates]

**Files I'm Providing**:
1. `PandaRobot.cs` - Original robot controller
2. `Chain.cs` - Kinematic chain (Panda version)
3. `Segment.cs` - Joint/segment representation (Panda version)
4. `Jacobian.cs`, `FastIterSolve.cs`, `IKJoint.cs` - Core IK components

**What I Need**:

1. **Analyze UR3e Adaptation**:
   - I'll provide both Panda and UR3e versions for comparison
   - Identify what changed and why
   - Understand the key patterns for adaptation

2. **Create [YOUR ROBOT] Implementation**:
   - Modify `Segment.cs`:
     - Replace hardcoded joint axes (lines 121-128) with computed axes using `anchorRotation`
     - Use the UR3e pattern (lines 110-123 in UR3e version)
   
   - Modify `Chain.cs`:
     - Add flexible end effector detection (support "TCP", "tool0", etc.)
     - Implement automatic TCP offset calculation
     - Fix Jacobian cross product order: `Cross(segAxis, cartDelta)`
     - Apply toolVector after FK completion
   
   - Create `[YourRobot]Robot.cs`:
     - Update joint limits for my robot
     - Set appropriate home position
     - Define workspace constraints specific to my robot
     - Remove Panda-specific dynamics constants (if I don't have this data)

3. **Provide**:
   - Modified source files
   - Explanation of changes made
   - Testing recommendations
   - Potential issues to watch for

**Key Adaptation Principles from UR3e Success**:
1. ✅ Compute joint axes from `anchorRotation` instead of hardcoding
2. ✅ Flexible end effectordetection with multiple name patterns
3. ✅ Automatic TCP offset calculation from transform hierarchy
4. ✅ Correct Jacobian math (axis × radius, not radius × axis)
5. ✅ Clean separation of robot-specific vs. generic IK code

**My Unity Setup**:
- Unity Version: [version]
- Robot imported via: [URDF Importer / manually created / etc.]
- ArticulationBody hierarchy named: [root name] → [joint names] → [TCP name]

Please help me create a working IK solver for my robot following the proven Panda → UR3e adaptation patterns!

### 🤖 PROMPT END

---

## Verification

### Checklist

- [ ] Joint axes computed correctly (debug log verification)
- [ ] TCP offset calculated automatically
- [ ] Forward kinematics matches expected results
- [ ] IK converges for reachable targets
- [ ] Joint limits enforced
- [ ] Workspace validation works
- [ ] No Unity errors in console
- [ ] Smooth motion without jittering

### Common Issues

**Issue**: IK doesn't converge
- **Check**: Joint limits match your robot's actual limits
- **Check**: Target is within reachable workspace
- **Check**: Initial guess is reasonable

**Issue**: TCP position is offset
- **Check**: TCP transform is correctly identified
- **Check**: `toolVector` calculation in Chain constructor
- **Check**: URDF import scale (Unity uses meters, check your units)

**Issue**: Robot moves to wrong position
- **Check**: Joint axis directions in `Segment.AddJoint()`
- **Check**: Jacobian cross product order
- **Check**: DH parameter consistency

**Issue**: Unstable motion
- **Check**: ArticulationBody stiffness/damping values
- **Check**: Velocity limits are reasonable
- **Check**: Joint drive parameters

---

## Success Criteria

Your adaptation is successful when:

1. ✅ FK produces correct TCP positions for known joint angles
2. ✅ IK converges reliably for reachable targets (within tolerance)
3. ✅ End effector reaches target pose accurately (position ± 1mm, orientation ± 1°)
4. ✅ Motion is smooth without excessive oscillation
5. ✅ No joint limit violations during operation
6. ✅ Workspace validation correctly identifies unreachable targets

---

## Example: UR3e Adaptation Summary

| Component | Change | Reason |
|-----------|--------|--------|
| **Segment.cs** | Hardcoded axes → Computed from `anchorRotation` | UR3e has different joint orientations than Panda |
| **Chain.cs** | Single EE name → Multiple EE patterns | UR models use "tool0", not "end_effector" |
| **Chain.cs** | Inline toolVector → Computed TCP offset | UR3e TCP is nested differently in hierarchy |
| **Chain.cs** | `Cross(r, axis)` → `Cross(axis, r)` | Mathematical correctness |
| **UR3eRobot.cs** | ±2π limits on all joints | UR3e has continuous rotation capability |
| **UR3eRobot.cs** | Simpler workspace (sphere) | UR3e has more uniform workspace |

**Result**: Fully functional IK solver for UR3e with no changes to core algorithm (`FastIterSolve.cs`, `Jacobian.cs`)

---

## Need Help?

- Review the UR3e adaptation as a reference example
- Compare Panda vs UR3e versions side-by-side
- Use the AI prompt above for assisted adaptation
- Check Unity console for debug logs from Chain constructor

Good luck with your adaptation! 🚀
