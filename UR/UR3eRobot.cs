using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// UR3e 机器人模型类
/// 
/// 提供 UR3e 机械臂的完整 IK 求解和控制功能：
/// - 6自由度关节控制
/// - 基于 Jacobian 的 IK 求解
/// - 关节限位约束
/// - 工作空间检查
/// 
/// UR3e 规格：
/// - 6个旋转关节 (全部为 Revolute)
/// - 最大伸展半径：约 500mm
/// - 重复定位精度：±0.03mm
/// - 有效载荷：3kg
/// </summary>
public class UR3eRobot
{
    #region UR3e 常量
    
    // UR3e 关节限位 (弧度) - 所有关节 ±360°
    // 注意：Unity ArticulationBody 使用度数
    private static readonly float[] MaxJointArrayRad = 
    { 
        2f * Mathf.PI,   // J1: base
        2f * Mathf.PI,   // J2: shoulder
        2f * Mathf.PI,   // J3: elbow
        2f * Mathf.PI,   // J4: wrist1
        2f * Mathf.PI,   // J5: wrist2
        2f * Mathf.PI    // J6: wrist3
    };
    
    private static readonly float[] MinJointArrayRad = 
    { 
        -2f * Mathf.PI,  // J1: base
        -2f * Mathf.PI,  // J2: shoulder
        -2f * Mathf.PI,  // J3: elbow
        -2f * Mathf.PI,  // J4: wrist1
        -2f * Mathf.PI,  // J5: wrist2
        -2f * Mathf.PI   // J6: wrist3
    };
    
    // UR3e DH 参数 (米) - 用于参考
    // d: [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
    // a: [0, -0.24365, -0.21325, 0, 0, 0]
    // alpha: [π/2, 0, 0, π/2, -π/2, 0]
    
    // 连杆质量 (kg) - 可选，用于动力学计算
    private static readonly float[] MassArray = { 2.0f, 3.42f, 1.26f, 0.8f, 0.8f, 0.35f };
    
    // 最大关节速度 (度/秒)
    public float VelocityLimitDeg = 180.0f;
    
    // 夹爪参数
    public float GripperOpen = 0.05f;
    public float GripperClosed = 0.0f;
    public float GripperGoal = 0.0f;
    
    #endregion
    
    #region 公共属性
    
    /// <summary>
    /// 运动学链
    /// </summary>
    public Chain Articulation { get; private set; }
    
    /// <summary>
    /// IK 求解器
    /// </summary>
    public FastIterSolve IKSolver { get; private set; } = new FastIterSolve();
    
    /// <summary>
    /// 初始关节配置 (度)
    /// </summary>
    public Vector<float> InitialJointState { get; private set; }
    
    /// <summary>
    /// 目标关节状态 (度)
    /// </summary>
    public Vector<float> GoalJointState { get; set; }
    
    /// <summary>
    /// 当前关节速度 (度/秒)
    /// </summary>
    public Vector<float> JointVelocity { get; private set; }
    
    /// <summary>
    /// 当前关节加速度 (度/秒²)
    /// </summary>
    public Vector<float> JointAcceleration { get; private set; }
    
    /// <summary>
    /// 最后的目标末端位姿
    /// </summary>
    public Matrix4x4 LastGoalPose { get; private set; }
    
    /// <summary>
    /// IK 最大迭代次数
    /// </summary>
    public float MaxIKCycles = 10000;
    
    #endregion
    
    #region 工作空间参数
    
    // UR3e 工作空间限制 (米)
    private const float UR3e_MaxReach = 0.5f;        // 最大伸展半径
    private const float UR3e_MinReach = 0.15f;       // 最小伸展半径（避免自碰撞）
    private const float UR3e_MaxHeight = 0.6f;       // 最大高度（相对于基座）
    private const float UR3e_MinHeight = -0.2f;      // 最小高度
    
    #endregion
    
    #region 构造函数
    
    /// <summary>
    /// 从 Unity GameObject 构建 UR3e 机器人模型
    /// </summary>
    /// <param name="robotBase">机器人根对象（包含 ArticulationBody 链）</param>
    /// <param name="customJointMin">自定义关节下限（度，可选）</param>
    /// <param name="customJointMax">自定义关节上限（度，可选）</param>
    public UR3eRobot(GameObject robotBase, Vector<float> customJointMin = null, Vector<float> customJointMax = null)
    {
        // 使用自定义限位或默认 UR3e 限位
        Vector<float> jointMin = customJointMin ?? 
            Mathf.Rad2Deg * Vector<float>.Build.DenseOfArray(MinJointArrayRad);
        Vector<float> jointMax = customJointMax ?? 
            Mathf.Rad2Deg * Vector<float>.Build.DenseOfArray(MaxJointArrayRad);
        
        // 创建运动学链
        Articulation = new Chain(robotBase, jointMin, jointMax, VelocityLimitDeg);
        
        // 验证关节数量
        if (Articulation.numberJoints != 6)
        {
            Debug.LogWarning($"[UR3eRobot] Expected 6 joints for UR3e, but found {Articulation.numberJoints}");
        }
        
        // 初始化关节状态向量
        int numJoints = Articulation.numberJoints;
        InitialJointState = Vector<float>.Build.Dense(numJoints);
        GoalJointState = Vector<float>.Build.Dense(numJoints);
        JointVelocity = Vector<float>.Build.Dense(numJoints);
        JointAcceleration = Vector<float>.Build.Dense(numJoints);
        
        // 设置 UR3e 默认 Home 姿态 (度)
        // 这是一个常见的工作姿态，手臂向前伸展
        SetDefaultHomePosition();
        
        // 设置夹爪目标为打开状态
        GripperGoal = GripperOpen;
        
        Debug.Log($"[UR3eRobot] Initialized with {numJoints} joints");
        Debug.Log($"[UR3eRobot] Root position: {Articulation.rootPosition}");
        Debug.Log($"[UR3eRobot] Tool offset: {Articulation.Gripper.toolVector}");
    }
    
    /// <summary>
    /// 设置默认 Home 位置
    /// </summary>
    private void SetDefaultHomePosition()
    {
        if (InitialJointState.Count >= 6)
        {
            // UR3e 典型 Home 姿态
            InitialJointState[0] = 0.0f;      // Base: 正前方
            InitialJointState[1] = -90.0f;    // Shoulder: 向下
            InitialJointState[2] = 90.0f;     // Elbow: 向上
            InitialJointState[3] = -90.0f;    // Wrist1
            InitialJointState[4] = -90.0f;    // Wrist2
            InitialJointState[5] = 0.0f;      // Wrist3
        }
    }
    
    #endregion
    
    #region IK 求解
    
    /// <summary>
    /// 求解逆运动学
    /// </summary>
    /// <param name="goalPose">目标末端位姿（世界坐标系）</param>
    /// <param name="initialGuess">初始猜测关节角（度）</param>
    /// <param name="weights">代价函数权重（可选）</param>
    /// <returns>求解后的关节角（度）</returns>
    public Vector<float> SolveInverseKinematics(Matrix4x4 goalPose, Vector<float> initialGuess, params float[] weights)
    {
        int cycles = 0;
        bool targetReached = false;
        
        Vector<float> qUpdate = Vector<float>.Build.Dense(Articulation.numberJoints);
        Vector<float> qOriginal = Vector<float>.Build.DenseOfVector(initialGuess);
        Vector<float> qCurrent = Vector<float>.Build.DenseOfVector(initialGuess);
        
        while (!targetReached)
        {
            // 执行一次 IK 迭代
            IKSolver.CartToJnt(Articulation, qCurrent, goalPose, qOriginal, out targetReached, weights)
                    .CopyTo(qUpdate);
            
            // 更新模拟运动学
            Articulation.UpdateChainKinematics(qUpdate);
            Articulation.JntToCart();
            
            // 更新当前状态
            qUpdate.CopyTo(qCurrent);
            cycles++;
            
            if (cycles > MaxIKCycles)
            {
                Debug.LogWarning($"[UR3eRobot] IK not converging after {cycles} iterations");
                Debug.Log($"[UR3eRobot] Current joint state: {qUpdate}");
                Debug.Log($"[UR3eRobot] Position error: {IKSolver.p_err}");
                break;
            }
        }
        
        if (targetReached)
        {
            Debug.Log($"[UR3eRobot] IK converged in {cycles} iterations");
        }
        
        LastGoalPose = goalPose;
        return qUpdate;
    }
    
    /// <summary>
    /// 使用当前关节状态作为初始猜测求解 IK
    /// </summary>
    public Vector<float> SolveInverseKinematics(Matrix4x4 goalPose)
    {
        return SolveInverseKinematics(goalPose, Articulation.jointState);
    }
    
    #endregion
    
    #region 关节控制
    
    /// <summary>
    /// 更新关节动力学状态（速度、加速度）
    /// </summary>
    public void UpdateJointDynamics()
    {
        for (int i = 0; i < Articulation.numberJoints; i++)
        {
            Segment seg;
            if (Articulation.chainMap.TryGetValue(i, out seg))
            {
                JointVelocity[i] = seg.linkedBody.jointVelocity[0];
                JointAcceleration[i] = seg.linkedBody.jointAcceleration[0];
            }
        }
    }
    
    /// <summary>
    /// 直接驱动关节到目标位置（会瞬移）
    /// </summary>
    /// <param name="goalJoints">目标关节角度（度）</param>
    public void DriveJoints(Vector<float> goalJoints)
    {
        int jointCount = 0;
        
        foreach (Segment seg in Articulation.segments)
        {
            if (seg.linkedBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                ArticulationDrive drive = seg.linkedBody.xDrive;
                
                float currentAngleDeg = Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                float deltaAngle = goalJoints[jointCount] - currentAngleDeg;
                
                // 只有当差异大于阈值时才更新目标
                if (Mathf.Abs(deltaAngle) > 0.05f)
                {
                    drive.target = goalJoints[jointCount];
                    seg.linkedBody.xDrive = drive;
                }
                
                jointCount++;
                
                if (jointCount >= Articulation.numberJoints) break;
            }
        }
    }
    
    /// <summary>
    /// 增量驱动关节到目标位置（平滑运动）
    /// 每次调用只移动一小步，需要在 FixedUpdate 中持续调用直到到达目标
    /// </summary>
    /// <param name="goalJoints">目标关节角度（度）</param>
    /// <param name="maxVelocity">最大关节速度（度/秒），默认使用 VelocityLimitDeg</param>
    /// <returns>是否已到达目标（所有关节误差小于阈值）</returns>
    public bool DriveJointsIncremental(Vector<float> goalJoints, float maxVelocity = -1f)
    {
        if (maxVelocity < 0) maxVelocity = VelocityLimitDeg;
        
        int jointCount = 0;
        bool allReached = true;
        float threshold = 0.5f; // 0.5度阈值认为到达
        
        foreach (Segment seg in Articulation.segments)
        {
            if (seg.linkedBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                ArticulationDrive drive = seg.linkedBody.xDrive;
                
                float currentAngleDeg = Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                float deltaAngle = goalJoints[jointCount] - currentAngleDeg;
                
                // 检查是否到达
                if (Mathf.Abs(deltaAngle) > threshold)
                {
                    allReached = false;
                    
                    // 计算本帧最大移动量
                    float maxDelta = maxVelocity * Time.fixedDeltaTime;
                    
                    // 限制移动量，保持方向
                    float actualDelta = Mathf.Sign(deltaAngle) * Mathf.Min(maxDelta, Mathf.Abs(deltaAngle));
                    
                    // 从当前位置增量移动
                    float newTarget = currentAngleDeg + actualDelta;
                    
                    drive.target = newTarget;
                    seg.linkedBody.xDrive = drive;
                }
                
                jointCount++;
                
                if (jointCount >= Articulation.numberJoints) break;
            }
        }
        
        return allReached;
    }
    
    /// <summary>
    /// 移动到 Home 位置
    /// </summary>
    public void MoveToHome()
    {
        DriveJoints(InitialJointState);
    }
    
    /// <summary>
    /// 设置自定义 Home 位置
    /// </summary>
    public void SetHomePosition(Vector<float> homeJoints)
    {
        if (homeJoints.Count == Articulation.numberJoints)
        {
            homeJoints.CopyTo(InitialJointState);
        }
        else
        {
            Debug.LogError($"[UR3eRobot] Home position must have {Articulation.numberJoints} elements");
        }
    }
    
    #endregion
    
    #region 工作空间检查
    
    /// <summary>
    /// 检查位置是否在 UR3e 工作空间内
    /// </summary>
    /// <param name="inputPose">目标位姿</param>
    /// <returns>true 如果在工作空间内</returns>
    public bool InsideWorkspace(Matrix4x4 inputPose)
    {
        Vector3 targetPos = new Vector3(inputPose.m03, inputPose.m13, inputPose.m23);
        return InsideWorkspace(targetPos);
    }
    
    /// <summary>
    /// 检查位置是否在 UR3e 工作空间内
    /// </summary>
    /// <param name="worldPosition">世界坐标位置</param>
    /// <returns>true 如果在工作空间内</returns>
    public bool InsideWorkspace(Vector3 worldPosition)
    {
        // 转换到基座坐标系
        Vector3 localPos = worldPosition - Articulation.rootPosition;
        
        // 计算水平距离（XZ平面）
        float horizontalDist = new Vector2(localPos.x, localPos.z).magnitude;
        
        // 检查水平距离
        if (horizontalDist > UR3e_MaxReach)
        {
            Debug.LogWarning($"[UR3eRobot] Position {worldPosition} exceeds max reach ({horizontalDist:F3}m > {UR3e_MaxReach}m)");
            return false;
        }
        
        if (horizontalDist < UR3e_MinReach && Mathf.Abs(localPos.y) < 0.1f)
        {
            Debug.LogWarning($"[UR3eRobot] Position {worldPosition} too close to base (potential self-collision)");
            return false;
        }
        
        // 检查高度
        if (localPos.y > UR3e_MaxHeight)
        {
            Debug.LogWarning($"[UR3eRobot] Position {worldPosition} exceeds max height ({localPos.y:F3}m > {UR3e_MaxHeight}m)");
            return false;
        }
        
        if (localPos.y < UR3e_MinHeight)
        {
            Debug.LogWarning($"[UR3eRobot] Position {worldPosition} below min height ({localPos.y:F3}m < {UR3e_MinHeight}m)");
            return false;
        }
        
        return true;
    }
    
    /// <summary>
    /// 检查是否可在速度限制内到达目标位姿
    /// </summary>
    /// <param name="inputPose">目标位姿</param>
    /// <param name="dt">时间步长</param>
    /// <returns>true 如果可达</returns>
    public bool InsideVelocityLimits(Matrix4x4 inputPose, float dt)
    {
        // 计算位姿误差
        Vector<float> poseError = IKSolver.CalcPoseError(Articulation.base2EETransform, inputPose);
        Vector<float> dX = poseError / dt;
        
        // 计算所需关节速度
        Jacobian cJ = Articulation.BuildJacobian(Articulation.jointState, 1.0f);
        cJ.SVDPseudoInverse();
        
        Vector<float> dTheta = cJ.inverse * dX;
        
        // 检查是否超过速度限制
        float maxVel = Mathf.Abs(dTheta.Maximum());
        if (maxVel > VelocityLimitDeg)
        {
            Debug.LogWarning($"[UR3eRobot] Required velocity ({maxVel:F1}°/s) exceeds limit ({VelocityLimitDeg}°/s)");
            return false;
        }
        
        return true;
    }
    
    #endregion
    
    #region 状态查询
    
    /// <summary>
    /// 获取当前末端执行器位姿
    /// </summary>
    public Matrix4x4 GetCurrentEndEffectorPose()
    {
        return Articulation.base2EETransform;
    }
    
    /// <summary>
    /// 获取当前末端执行器位置
    /// </summary>
    public Vector3 GetCurrentEndEffectorPosition()
    {
        Matrix4x4 eePose = Articulation.base2EETransform;
        return new Vector3(eePose.m03, eePose.m13, eePose.m23);
    }
    
    /// <summary>
    /// 获取当前末端执行器旋转
    /// </summary>
    public Quaternion GetCurrentEndEffectorRotation()
    {
        return Articulation.base2EETransform.rotation;
    }
    
    /// <summary>
    /// 获取当前关节状态（度）
    /// </summary>
    public Vector<float> GetCurrentJointState()
    {
        Articulation.JointUpdate();
        return Vector<float>.Build.DenseOfVector(Articulation.jointState);
    }
    
    /// <summary>
    /// 同步关节状态（从物理引擎读取）
    /// </summary>
    public void SyncJointState()
    {
        Articulation.JointUpdate();
    }
    
    #endregion
    
    #region 工具方法
    
    /// <summary>
    /// 创建位姿矩阵
    /// </summary>
    public static Matrix4x4 CreatePose(Vector3 position, Quaternion rotation)
    {
        return Matrix4x4.TRS(position, rotation, Vector3.one);
    }
    
    /// <summary>
    /// 从位姿矩阵提取位置
    /// </summary>
    public static Vector3 GetPosition(Matrix4x4 pose)
    {
        return new Vector3(pose.m03, pose.m13, pose.m23);
    }
    
    /// <summary>
    /// 打印关节状态（调试用）
    /// </summary>
    public void PrintJointState()
    {
        StringBuilder sb = new StringBuilder();
        sb.Append("[UR3eRobot] Joint State (deg): [");
        for (int i = 0; i < Articulation.jointState.Count; i++)
        {
            sb.Append($"{Articulation.jointState[i]:F1}");
            if (i < Articulation.jointState.Count - 1) sb.Append(", ");
        }
        sb.Append("]");
        Debug.Log(sb.ToString());
    }
    
    #endregion
}
