using System;
using System.Collections;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Pick & Place 测试管理器
/// 
/// 管理实体抓取测试的完整流程：
/// - 自动生成待抓取物体
/// - 协调 IK 运动和夹爪控制
/// - 执行 Pick & Place 序列
/// </summary>
public class PickPlaceManager : MonoBehaviour
{
    #region Inspector 配置
    
    [Header("Robot References")]
    [Tooltip("IK 测试器（用于机械臂控制）")]
    public UR3eIKTester ikTester;
    
    [Tooltip("夹爪控制器")]
    public PincherController pincherController;
    
    [Tooltip("抓取控制器")]
    public GripperGraspController graspController;
    
    [Header("Pick & Place Positions (相对于机器人基座)")]
    [Tooltip("拾取位置")]
    public Vector3 pickPosition = new Vector3(-0.3f, 0.05f, -0.3f);
    
    [Tooltip("放置位置")]
    public Vector3 placePosition = new Vector3(0.3f, 0.05f, -0.3f);
    
    [Tooltip("接近高度（相对于 pick/place 位置）")]
    public float approachHeight = 0.15f;
    
    [Tooltip("朝下方向 (Euler角度)")]
    public Vector3 downwardOrientation = new Vector3(-90f, 0f, 0f);
    
    [Header("Motion Settings")]
    [Tooltip("关节运动速度（度/秒）")]
    public float jointSpeed = 60f;
    
    [Tooltip("步骤之间的停顿时间（秒）")]
    public float stepPause = 0.5f;
    
    [Tooltip("夹爪动作等待时间（秒）")]
    public float gripperActionTime = 0.8f;
    
    [Header("Object Settings")]
    [Tooltip("物体形状")]
    public ObjectShape objectShape = ObjectShape.Cube;
    
    [Tooltip("物体尺寸（米）")]
    public float objectSize = 0.03f;
    
    [Tooltip("物体质量（千克）")]
    public float objectMass = 0.1f;
    
    [Tooltip("物体颜色")]
    public Color objectColor = new Color(0.2f, 0.6f, 1f);
    
    [Tooltip("物体距离地面的高度（用于生成时的 Y 偏移）")]
    public float objectHeightOffset = 0.02f;
    
    [Header("Debug")]
    [Tooltip("详细日志")]
    public bool verboseLog = true;
    
    #endregion
    
    #region 枚举定义
    
    public enum ObjectShape
    {
        Cube,
        Sphere,
        Cylinder
    }
    
    public enum TestState
    {
        Idle,
        MovingToPickApproach,
        MovingToPick,
        Grasping,
        LiftingFromPick,
        MovingToPlaceApproach,
        MovingToPlace,
        Releasing,
        LiftingFromPlace,
        Completed,
        Failed
    }
    
    #endregion
    
    #region 公共属性
    
    /// <summary>
    /// 当前测试状态
    /// </summary>
    public TestState CurrentState { get; private set; } = TestState.Idle;
    
    /// <summary>
    /// 是否正在运行测试
    /// </summary>
    public bool IsRunning { get; private set; } = false;
    
    /// <summary>
    /// 当前测试物体
    /// </summary>
    public GraspableObject CurrentObject { get; private set; } = null;
    
    #endregion
    
    #region 私有变量
    
    private UR3eRobot robot;
    private Coroutine testCoroutine;
    private Vector3 robotBasePosition;
    
    // 测试统计
    private int totalAttempts = 0;
    private int successfulPicks = 0;
    private int successfulPlaces = 0;
    
    #endregion
    
    #region Unity 生命周期
    
    void Start()
    {
        // 自动查找组件
        FindReferences();
    }
    
    void FindReferences()
    {
        // 查找 IK 测试器
        if (ikTester == null)
        {
            ikTester = FindObjectOfType<UR3eIKTester>();
        }
        
        // 查找夹爪控制器
        if (pincherController == null)
        {
            pincherController = FindObjectOfType<PincherController>();
        }
        
        // 查找抓取控制器
        if (graspController == null)
        {
            graspController = FindObjectOfType<GripperGraspController>();
        }
        
        // 验证
        if (ikTester == null)
            {
            Debug.LogWarning("[PickPlaceManager] UR3eIKTester not found!");
        }
        if (pincherController == null)
        {
            Debug.LogWarning("[PickPlaceManager] PincherController not found!");
        }
    }
    
    #endregion
    
    #region 公共方法
    
    /// <summary>
    /// 开始 Pick & Place 测试
    /// </summary>
    public void StartTest()
    {
        if (IsRunning)
        {
            Log("Test already running!");
            return;
        }
        
        // 验证配置
        if (!ValidateConfiguration())
        {
            Log("Configuration validation failed!");
            return;
        }
        
        // 获取机器人引用
        robot = ikTester.GetRobot();
        if (robot == null)
        {
            Log("Failed to get robot reference!");
            return;
        }
        
        robotBasePosition = robot.Articulation.rootPosition;
        
        // 生成物体
        SpawnObject();
        
        // 开始测试序列
        testCoroutine = StartCoroutine(RunPickPlaceSequence());
    }
    
    /// <summary>
    /// 停止当前测试
    /// </summary>
    public void StopTest()
    {
        if (testCoroutine != null)
        {
            StopCoroutine(testCoroutine);
            testCoroutine = null;
        }
        
        IsRunning = false;
        CurrentState = TestState.Idle;
        
        // 打开夹爪
        if (pincherController != null)
        {
            pincherController.gripState = GripState.Opening;
        }
        
        Log("Test stopped");
    }
    
    /// <summary>
    /// 重置测试（销毁物体、重置状态）
    /// </summary>
    public void ResetTest()
    {
        StopTest();
        
        // 销毁物体
        if (CurrentObject != null)
        {
            Destroy(CurrentObject.gameObject);
            CurrentObject = null;
        }
        
        // 重置夹爪
        if (pincherController != null)
        {
            pincherController.ResetGripToOpen();
        }
        
        // 重置抓取控制器
        if (graspController != null)
        {
            graspController.ForceRelease();
        }
        
        CurrentState = TestState.Idle;
        
        Log("Test reset");
    }
    
    /// <summary>
    /// 仅生成物体（不执行抓取）
    /// </summary>
    public void SpawnObjectOnly()
    {
        // 销毁旧物体
        if (CurrentObject != null)
        {
            Destroy(CurrentObject.gameObject);
        }
        
        SpawnObject();
    }
    
    #endregion
    
    #region 测试序列
    
    /// <summary>
    /// 执行完整的 Pick & Place 序列
    /// </summary>
    IEnumerator RunPickPlaceSequence()
    {
        IsRunning = true;
        totalAttempts++;
        
        Log("========== Starting Physical Pick & Place ==========");
        Log($"Pick position: {pickPosition}");
        Log($"Place position: {placePosition}");
        Log($"Object: {objectShape}, size={objectSize}m, mass={objectMass}kg");
        
        Quaternion downRotation = Quaternion.Euler(downwardOrientation);
        
        // === Phase 1: Pick ===
        
        // Step 1: 移动到 Pick 位置上方
        CurrentState = TestState.MovingToPickApproach;
        Vector3 pickApproach = robotBasePosition + pickPosition + Vector3.up * approachHeight;
        Log($"Step 1: Moving to pick approach - {pickApproach}");
        
        yield return StartCoroutine(MoveToPosition(pickApproach, downRotation));
        if (!CheckMoveSuccess()) { yield return HandleFailure("Failed to reach pick approach"); yield break; }
        
        yield return new WaitForSeconds(stepPause);
        
        // Step 2: 下降到 Pick 位置
        CurrentState = TestState.MovingToPick;
        Vector3 pickPos = robotBasePosition + pickPosition;
        Log($"Step 2: Moving to pick position - {pickPos}");
        
        yield return StartCoroutine(MoveToPosition(pickPos, downRotation));
        if (!CheckMoveSuccess()) { yield return HandleFailure("Failed to reach pick position"); yield break; }
        
        yield return new WaitForSeconds(stepPause * 0.5f);
        
        // Step 3: 闭合夹爪抓取
        CurrentState = TestState.Grasping;
        Log("Step 3: Closing gripper...");
        
        yield return StartCoroutine(CloseGripper());
        yield return new WaitForSeconds(gripperActionTime);
        
        // 检查是否成功抓取
        bool graspSuccess = graspController != null && graspController.IsGrasping;
        if (graspSuccess)
        {
            successfulPicks++;
            Log("Grasp successful!");
        }
        else
        {
            Log("Warning: Grasp may have failed (no object detected)");
            // 继续执行，可能物体仍然被物理方式夹住
        }
        
        // Step 4: 抬起
        CurrentState = TestState.LiftingFromPick;
        Vector3 pickLift = robotBasePosition + pickPosition + Vector3.up * approachHeight;
        Log($"Step 4: Lifting from pick - {pickLift}");
        
        yield return StartCoroutine(MoveToPosition(pickLift, downRotation));
        yield return new WaitForSeconds(stepPause);
        
        // === Phase 2: Place ===
        
        // Step 5: 移动到 Place 位置上方
        CurrentState = TestState.MovingToPlaceApproach;
        Vector3 placeApproach = robotBasePosition + placePosition + Vector3.up * approachHeight;
        Log($"Step 5: Moving to place approach - {placeApproach}");
        
        yield return StartCoroutine(MoveToPosition(placeApproach, downRotation));
        if (!CheckMoveSuccess()) { yield return HandleFailure("Failed to reach place approach"); yield break; }
        
        yield return new WaitForSeconds(stepPause);
        
        // Step 6: 下降到 Place 位置
        CurrentState = TestState.MovingToPlace;
        Vector3 placePos = robotBasePosition + placePosition;
        Log($"Step 6: Moving to place position - {placePos}");
        
        yield return StartCoroutine(MoveToPosition(placePos, downRotation));
        yield return new WaitForSeconds(stepPause * 0.5f);
        
        // Step 7: 打开夹爪释放
        CurrentState = TestState.Releasing;
        Log("Step 7: Opening gripper...");
        
        yield return StartCoroutine(OpenGripper());
        yield return new WaitForSeconds(gripperActionTime);
        
        // 检查物体是否在正确位置
        if (CurrentObject != null)
        {
            float distanceToPlace = Vector3.Distance(
                CurrentObject.transform.position,
                robotBasePosition + placePosition
            );
            
            if (distanceToPlace < 0.1f) // 10cm 容忍
            {
                successfulPlaces++;
                Log($"Place successful! Distance to target: {distanceToPlace * 100:F1}cm");
            }
            else
            {
                Log($"Warning: Object may not be at target. Distance: {distanceToPlace * 100:F1}cm");
            }
        }
        
        // Step 8: 抬起离开
        CurrentState = TestState.LiftingFromPlace;
        Vector3 placeLift = robotBasePosition + placePosition + Vector3.up * approachHeight;
        Log($"Step 8: Lifting from place - {placeLift}");
        
        yield return StartCoroutine(MoveToPosition(placeLift, downRotation));
        yield return new WaitForSeconds(stepPause);
        
        // === Complete ===
        CurrentState = TestState.Completed;
        IsRunning = false;
        
        Log("========== Pick & Place Complete ==========");
        Log($"Statistics: Attempts={totalAttempts}, Picks={successfulPicks}, Places={successfulPlaces}");
    }
    
    #endregion
    
    #region 运动控制
    
    /// <summary>
    /// 移动到指定位置
    /// </summary>
    IEnumerator MoveToPosition(Vector3 position, Quaternion rotation)
    {
        // 检查工作空间
        if (!robot.InsideWorkspace(position))
        {
            Log($"Position {position} outside workspace!");
            yield break;
        }
        
        // 创建目标位姿
        Matrix4x4 targetPose = Matrix4x4.TRS(position, rotation, Vector3.one);
        
        // 求解 IK
        robot.SyncJointState();
        Vector<float> initialGuess = robot.GetCurrentJointState();
        Vector<float> solution = robot.SolveInverseKinematics(targetPose, initialGuess);
        
        // 检查求解结果
        Vector<float> posError = robot.IKSolver.p_err;
        float positionError = Mathf.Sqrt(posError[0] * posError[0] + posError[1] * posError[1] + posError[2] * posError[2]);
        
        if (positionError > 0.01f)
        {
            Log($"IK error too large: {positionError * 1000:F1}mm");
            yield break;
        }
        
        // 执行运动
        yield return StartCoroutine(MoveToJoints(solution));
    }
    
    /// <summary>
    /// 移动到指定关节位置
    /// </summary>
    IEnumerator MoveToJoints(Vector<float> targetJoints)
    {
        float timeout = 30f;
        float startTime = Time.time;
        
        while (Time.time - startTime < timeout)
        {
            bool reached = robot.DriveJointsIncremental(targetJoints, jointSpeed);
            
            if (reached)
            {
                yield return new WaitForSeconds(0.1f);
                break;
            }
            
            yield return new WaitForFixedUpdate();
        }
    }
    
    /// <summary>
    /// 闭合夹爪
    /// </summary>
    IEnumerator CloseGripper()
    {
        if (pincherController != null)
        {
            pincherController.gripState = GripState.Closing;
            
            // 等待夹爪闭合到阈值
            float timeout = 3f;
            float startTime = Time.time;
            
            while (Time.time - startTime < timeout)
            {
                if (pincherController.CurrentGrip() > 0.7f)
                {
                    break;
                }
                yield return new WaitForFixedUpdate();
            }
            
            pincherController.gripState = GripState.Fixed;
        }
    }
    
    /// <summary>
    /// 打开夹爪
    /// </summary>
    IEnumerator OpenGripper()
    {
        if (pincherController != null)
        {
        pincherController.gripState = GripState.Opening;
        
        // 等待夹爪打开
        float timeout = 3f;
        float startTime = Time.time;
        
        while (Time.time - startTime < timeout)
        {
                if (pincherController.CurrentGrip() < 0.1f)
            {
                break;
            }
            yield return new WaitForFixedUpdate();
        }
        
        pincherController.gripState = GripState.Fixed;
    }
    }
    
    #endregion
    
    #region 物体生成
    
    /// <summary>
    /// 在 Pick 位置生成物体
    /// </summary>
    void SpawnObject()
    {
        // 计算生成位置（Pick 位置 + 高度偏移）
        Vector3 spawnPos = robotBasePosition + pickPosition;
        spawnPos.y = objectHeightOffset + objectSize / 2f; // 确保物体在地面上方
        
        // 根据形状创建
        switch (objectShape)
        {
            case ObjectShape.Cube:
                CurrentObject = GraspableObject.CreateCube(spawnPos, objectSize, objectMass, objectColor);
                break;
            case ObjectShape.Sphere:
                CurrentObject = GraspableObject.CreateSphere(spawnPos, objectSize, objectMass, objectColor);
                break;
            case ObjectShape.Cylinder:
                // 使用 Cube 作为替代
                CurrentObject = GraspableObject.CreateCube(spawnPos, objectSize, objectMass, objectColor);
                break;
        }
        
        Log($"Spawned {objectShape} at {spawnPos}");
    }
    
    #endregion
    
    #region 辅助方法
    
    /// <summary>
    /// 验证配置
    /// </summary>
    bool ValidateConfiguration()
    {
        if (ikTester == null)
        {
            FindReferences();
        }
        
        if (ikTester == null)
        {
            Debug.LogError("[PickPlaceManager] UR3eIKTester is required!");
            return false;
        }
        
        if (pincherController == null)
        {
            Debug.LogWarning("[PickPlaceManager] PincherController not found - gripper control disabled");
        }
        
        return true;
    }
    
    /// <summary>
    /// 检查移动是否成功
    /// </summary>
    bool CheckMoveSuccess()
    {
        // 简单检查：如果 IK 误差在可接受范围内就认为成功
        if (robot != null && robot.IKSolver.p_err != null)
        {
            Vector<float> err = robot.IKSolver.p_err;
            float posErr = Mathf.Sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
            return posErr < 0.02f; // 20mm
        }
        return true;
        }
        
    /// <summary>
    /// 处理失败
    /// </summary>
    IEnumerator HandleFailure(string message)
        {
        Log($"FAILED: {message}");
        CurrentState = TestState.Failed;
        IsRunning = false;
        
        // 打开夹爪
        yield return StartCoroutine(OpenGripper());
    }
    
    /// <summary>
    /// 日志输出
    /// </summary>
    void Log(string message)
    {
        if (verboseLog)
        {
            Debug.Log($"[PickPlaceManager] {message}");
        }
    }
    
    #endregion
    
    #region GUI 显示
    
    void OnGUI()
    {
        if (!IsRunning && CurrentState == TestState.Idle) return;
        
        GUIStyle boxStyle = new GUIStyle(GUI.skin.box);
        boxStyle.normal.background = MakeTexture(2, 2, new Color(0f, 0f, 0f, 0.8f));
        
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 14;
        labelStyle.normal.textColor = Color.white;
        
        GUILayout.BeginArea(new Rect(Screen.width - 320, 200, 310, 200), boxStyle);
        
        GUILayout.Label("Pick & Place Status", new GUIStyle(labelStyle) { fontStyle = FontStyle.Bold, fontSize = 16 });
        GUILayout.Space(10);
        
        string stateColor = CurrentState == TestState.Completed ? "<color=green>" :
                           CurrentState == TestState.Failed ? "<color=red>" : "<color=yellow>";
        GUILayout.Label($"State: {stateColor}{CurrentState}</color>", labelStyle);
        
        GUILayout.Label($"Object: {(CurrentObject != null ? CurrentObject.gameObject.name : "None")}", labelStyle);
        
        if (graspController != null)
        {
            GUILayout.Label($"Grasping: {graspController.IsGrasping}", labelStyle);
        }
        
        if (pincherController != null)
        {
            GUILayout.Label($"Grip: {pincherController.CurrentGrip():F2}", labelStyle);
        }
        
        GUILayout.Space(10);
        GUILayout.Label($"Attempts: {totalAttempts}", labelStyle);
        GUILayout.Label($"Successful Picks: {successfulPicks}", labelStyle);
        GUILayout.Label($"Successful Places: {successfulPlaces}", labelStyle);
        
        GUILayout.EndArea();
    }
    
    Texture2D MakeTexture(int width, int height, Color color)
    {
        Color[] pixels = new Color[width * height];
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = color;
        }
        Texture2D texture = new Texture2D(width, height);
        texture.SetPixels(pixels);
        texture.Apply();
        return texture;
    }
    
    #endregion
}
