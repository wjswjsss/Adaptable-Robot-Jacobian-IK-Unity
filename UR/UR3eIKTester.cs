using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// UR3e IK Solver 测试脚本
/// 
/// 用于测试 Copies/ 文件夹中的 Jacobian IK Solver
/// 
/// 键盘控制：
/// - [1] 测试1: 移动到 Home 位置
/// - [2] 测试2: 前向伸展测试
/// - [3] 测试3: 方形轨迹测试
/// - [4] 测试4: 随机可达点测试
/// - [5] FK验证: 验证正运动学计算
/// - [6] 轴向调试: 打印各关节轴向信息
/// - [R] 重置到初始状态
/// - [P] 打印当前状态
/// - [L] 输出日志到文件
/// </summary>
public class UR3eIKTester : MonoBehaviour
{
    #region Inspector 配置
    
    [Header("Robot References")]
    [Tooltip("UR3e 根对象（包含 ArticulationBody 链）")]
    public GameObject robotRoot;
    
    [Tooltip("末端执行器名称（用于停止链遍历）")]
    public string endEffectorName = "end_effector";
    
    [Tooltip("TCP Transform（用于FK验证，从Hierarchy拖入）")]
    public Transform tcpTransform;
    
    [Header("IK Parameters")]
    [Tooltip("最大迭代次数")]
    public int maxIterations = 10000;
    
    [Tooltip("收敛阈值")]
    public float convergenceThreshold = 1e-4f;
    
    [Header("Test Parameters")]
    [Tooltip("动作执行速度（度/秒）")]
    public float jointSpeed = 60f;
    
    [Tooltip("测试点之间的停顿时间（秒）")]
    public float pauseBetweenTests = 1.0f;
    
    [Tooltip("使用平滑运动（增量驱动）而非瞬移")]
    public bool useSmoothMotion = true;
    
    [Header("Pick & Place Settings")]
    [Tooltip("拾取位置 (相对于机器人基座)")]
    public Vector3 pickPosition = new Vector3(-0.3f, 0.2f, -0.4f);
    
    [Tooltip("放置位置 (相对于机器人基座)")]
    public Vector3 placePosition = new Vector3(0.3f, 0.2f, -0.4f);
    
    [Tooltip("抬升高度")]
    public float liftHeight = 0.15f;
    
    [Tooltip("朝下方向 (Euler角度): -90,0,0 使工具Z轴指向-Y(世界下方)")]
    public Vector3 downwardOrientation = new Vector3(-90f, 0f, 0f);
    
    [Header("Visualization")]
    [Tooltip("是否显示目标位置可视化")]
    public bool showTargetVisualization = true;
    
    [Tooltip("目标位置球体大小")]
    public float targetSphereSize = 0.02f;
    
    [Tooltip("目标轴线长度")]
    public float targetAxisLength = 0.1f;
    
    [Header("Debug")]
    [Tooltip("日志输出路径")]
    public string logFilePath = "Assets/Scripts/JacobianIK/Debug/CopiesIK_Test_Log.txt";
    
    [Tooltip("是否在控制台输出详细信息")]
    public bool verboseConsole = true;
    
    [Header("Physical Pick & Place")]
    [Tooltip("实体抓取管理器（如果为空会自动创建）")]
    public PickPlaceManager pickPlaceManager;
    
    [Tooltip("夹爪抓取控制器（挂载在 HandE 上）")]
    public GripperGraspController graspController;
    
    #endregion
    
    #region 私有变量
    
    private UR3eRobot robot;
    
    /// <summary>
    /// 获取机器人引用（供外部使用，如 PickPlaceManager）
    /// </summary>
    public UR3eRobot GetRobot() => robot;
    private StringBuilder logBuilder = new StringBuilder();
    private bool isTestRunning = false;
    private Coroutine currentTestCoroutine;
    
    // 测试统计
    private int totalTests = 0;
    private int passedTests = 0;
    private float totalSolveTime = 0f;
    
    // 协程结果传递（因为 yield return 不能直接返回 bool）
    private bool lastMoveSuccess = false;
    
    // 可视化
    private Vector3 currentTargetPosition;
    private Quaternion currentTargetRotation;
    private bool hasActiveTarget = false;
    
    #endregion
    
    #region Unity 生命周期
    
    void Start()
    {
        InitializeRobot();
        
        LogMessage("========================================");
        LogMessage("UR3e IK Tester Initialized");
        LogMessage($"Date: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        LogMessage("========================================");
        LogMessage("");
        LogMessage("Keyboard Controls:");
        LogMessage("  [1] Test 1: Move to Home");
        LogMessage("  [2] Test 2: Forward Reach");
        LogMessage("  [3] Test 3: Square Trajectory");
        LogMessage("  [4] Test 4: Random Reachable Points");
        LogMessage("  [R] Reset to Initial");
        LogMessage("  [P] Print Current State");
        LogMessage("  [L] Save Log to File");
        LogMessage("");
    }
    
    void Update()
    {
        HandleKeyboardInput();
    }
    
    #endregion
    
    #region 初始化
    
    void InitializeRobot()
    {
        if (robotRoot == null)
        {
            Debug.LogError("[UR3eIKTester] Robot root not assigned!");
            return;
        }
        
        try
        {
            robot = new UR3eRobot(robotRoot);
            LogMessage($"[Init] Robot initialized with {robot.Articulation.numberJoints} joints");
            LogMessage($"[Init] Root position: {robot.Articulation.rootPosition}");
            LogMessage($"[Init] Tool offset: {robot.Articulation.Gripper.toolVector}");
            
            // 设置 IK 参数
            robot.MaxIKCycles = maxIterations;
            robot.IKSolver.converge_eps = convergenceThreshold;
            
            // 初始化运动学
            robot.Articulation.JointUpdate();
            
            LogMessage($"[Init] Initial EE position: {robot.GetCurrentEndEffectorPosition()}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[UR3eIKTester] Failed to initialize robot: {e.Message}");
            LogMessage($"[ERROR] Initialization failed: {e.Message}");
        }
    }
    
    #endregion
    
    #region 键盘输入处理
    
    void HandleKeyboardInput()
    {
        if (robot == null) return;
        
        // 测试控制
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            StartTest("Home Position", TestHomePosition());
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            StartTest("Forward Reach", TestForwardReach());
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            StartTest("Square Trajectory", TestSquareTrajectory());
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            StartTest("Random Points", TestRandomPoints());
        }
        else if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            RunFKValidation();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha6))
        {
            RunAxisDebug();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha7))
        {
            StartTest("Pick and Place", TestPickAndPlace());
        }
        else if (Input.GetKeyDown(KeyCode.Alpha8))
        {
            StartPhysicalPickPlace();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha9))
        {
            ResetPhysicalPickPlace();
        }
        
        // 其他控制
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetRobot();
        }
        else if (Input.GetKeyDown(KeyCode.P))
        {
            PrintCurrentState();
        }
        else if (Input.GetKeyDown(KeyCode.L))
        {
            SaveLogToFile();
        }
    }
    
    void StartTest(string testName, IEnumerator testCoroutine)
    {
        if (isTestRunning)
        {
            LogMessage($"[Warning] Test already running, please wait...");
            return;
        }
        
        LogMessage("");
        LogMessage($"========== Starting Test: {testName} ==========");
        currentTestCoroutine = StartCoroutine(RunTest(testName, testCoroutine));
    }
    
    IEnumerator RunTest(string testName, IEnumerator testCoroutine)
    {
        isTestRunning = true;
        float startTime = Time.realtimeSinceStartup;
        
        yield return testCoroutine;
        
        float duration = Time.realtimeSinceStartup - startTime;
        LogMessage($"[{testName}] Completed in {duration:F2}s");
        LogMessage($"========== End Test: {testName} ==========");
        LogMessage("");
        
        isTestRunning = false;
    }
    
    #endregion
    
    #region 测试用例
    
    /// <summary>
    /// 测试1: 移动到 Home 位置
    /// </summary>
    IEnumerator TestHomePosition()
    {
        totalTests++;
        LogMessage("[Test1] Moving to Home position...");
        
        Vector<float> homeJoints = robot.InitialJointState;
        LogMessage($"[Test1] Target joints (deg): [{FormatVector(homeJoints)}]");
        
        yield return MoveToJointPosition(homeJoints);
        
        // 验证
        robot.SyncJointState();
        Vector<float> currentJoints = robot.GetCurrentJointState();
        float error = (float)(currentJoints - homeJoints).L2Norm();
        
        bool passed = error < 5.0f; // 5度误差容忍
        LogMessage($"[Test1] Final joints (deg): [{FormatVector(currentJoints)}]");
        LogMessage($"[Test1] Joint error: {error:F2} deg - {(passed ? "PASS" : "FAIL")}");
        
        if (passed) passedTests++;
    }
    
    /// <summary>
    /// 测试2: 前向伸展测试
    /// </summary>
    IEnumerator TestForwardReach()
    {
        totalTests++;
        LogMessage("[Test2] Forward reach test...");
        
        // 定义几个前向位置
        Vector3[] testPositions = new Vector3[]
        {
            new Vector3(0.3f, 0.2f, 0.0f),   // 正前方
            new Vector3(0.35f, 0.15f, 0.0f), // 更远更低
            new Vector3(0.25f, 0.3f, 0.0f),  // 近一点高一点
        };
        
        Quaternion targetRotation = Quaternion.Euler(-90f, 0f, 0f); // 工具Z轴朝下(-Y)
        
        int successCount = 0;
        
        for (int i = 0; i < testPositions.Length; i++)
        {
            Vector3 targetPos = robot.Articulation.rootPosition + testPositions[i];
            LogMessage($"[Test2] Point {i + 1}/{testPositions.Length}: {targetPos}");
            
            yield return StartCoroutine(MoveToCartesianPosition(targetPos, targetRotation));
            if (lastMoveSuccess) successCount++;
            
            yield return new WaitForSeconds(pauseBetweenTests);
        }
        
        bool passed = successCount == testPositions.Length;
        LogMessage($"[Test2] Success: {successCount}/{testPositions.Length} - {(passed ? "PASS" : "FAIL")}");
        
        if (passed) passedTests++;
    }
    
    /// <summary>
    /// 测试3: 方形轨迹测试
    /// </summary>
    IEnumerator TestSquareTrajectory()
    {
        totalTests++;
        LogMessage("[Test3] Square trajectory test...");
        
        // 方形轨迹的四个角点（相对于基座）
        float size = 0.1f;  // 方形边长
        float height = 0.25f;
        float forward = 0.3f;
        
        Vector3[] corners = new Vector3[]
        {
            new Vector3(forward, height, -size/2),  // 左下
            new Vector3(forward, height, size/2),   // 右下
            new Vector3(forward, height + size, size/2),   // 右上
            new Vector3(forward, height + size, -size/2),  // 左上
        };
        
        Quaternion targetRotation = Quaternion.Euler(-90f, 0f, 0f); // 工具Z轴朝下(-Y)
        
        int successCount = 0;
        
        // 执行两圈
        for (int lap = 0; lap < 2; lap++)
        {
            LogMessage($"[Test3] Lap {lap + 1}/2");
            
            for (int i = 0; i < corners.Length; i++)
            {
                Vector3 targetPos = robot.Articulation.rootPosition + corners[i];
                LogMessage($"[Test3] Corner {i + 1}: {corners[i]}");
                
                yield return StartCoroutine(MoveToCartesianPosition(targetPos, targetRotation));
                if (lastMoveSuccess) successCount++;
                
                yield return new WaitForSeconds(pauseBetweenTests * 0.5f);
            }
        }
        
        int totalPoints = corners.Length * 2;
        bool passed = successCount >= totalPoints * 0.8f; // 80% 成功率
        LogMessage($"[Test3] Success: {successCount}/{totalPoints} - {(passed ? "PASS" : "FAIL")}");
        
        if (passed) passedTests++;
    }
    
    /// <summary>
    /// 测试4: 随机可达点测试
    /// </summary>
    IEnumerator TestRandomPoints()
    {
        totalTests++;
        LogMessage("[Test4] Random reachable points test...");
        
        int numPoints = 5;
        int successCount = 0;
        
        Quaternion targetRotation = Quaternion.Euler(-90f, 0f, 0f); // 工具Z轴朝下(-Y)
        
        for (int i = 0; i < numPoints; i++)
        {
            // 生成随机可达点（球坐标）
            float r = UnityEngine.Random.Range(0.2f, 0.4f);     // 距离
            float theta = UnityEngine.Random.Range(-60f, 60f);   // 水平角度
            float phi = UnityEngine.Random.Range(10f, 60f);      // 仰角
            
            float thetaRad = theta * Mathf.Deg2Rad;
            float phiRad = phi * Mathf.Deg2Rad;
            
            Vector3 localPos = new Vector3(
                r * Mathf.Cos(phiRad) * Mathf.Cos(thetaRad),
                r * Mathf.Sin(phiRad),
                r * Mathf.Cos(phiRad) * Mathf.Sin(thetaRad)
            );
            
            Vector3 targetPos = robot.Articulation.rootPosition + localPos;
            LogMessage($"[Test4] Random point {i + 1}/{numPoints}: {localPos} (r={r:F2}, θ={theta:F0}°, φ={phi:F0}°)");
            
            yield return StartCoroutine(MoveToCartesianPosition(targetPos, targetRotation));
            if (lastMoveSuccess) successCount++;
            
            yield return new WaitForSeconds(pauseBetweenTests);
        }
        
        bool passed = successCount >= numPoints * 0.6f; // 60% 成功率（随机点可能不可达）
        LogMessage($"[Test4] Success: {successCount}/{numPoints} - {(passed ? "PASS" : "FAIL")}");
        
        if (passed) passedTests++;
    }
    
    /// <summary>
    /// 测试7: Pick and Place 测试（抓空气）
    /// </summary>
    IEnumerator TestPickAndPlace()
    {
        totalTests++;
        LogMessage("[Test7] Pick and Place test...");
        LogMessage($"[Test7] Pick position: {pickPosition}");
        LogMessage($"[Test7] Place position: {placePosition}");
        LogMessage($"[Test7] Lift height: {liftHeight}");
        LogMessage($"[Test7] Downward orientation: {downwardOrientation}");
        
        Quaternion downRotation = Quaternion.Euler(downwardOrientation);
        Vector3 rootPos = robot.Articulation.rootPosition;
        
        int successCount = 0;
        int totalSteps = 6;
        
        // Step 1: 移动到 Pick 位置上方（接近）
        Vector3 pickApproach = rootPos + pickPosition + Vector3.up * liftHeight;
        LogMessage($"[Test7] Step 1/6: Approach pick position - {pickApproach}");
        yield return StartCoroutine(MoveToCartesianPosition(pickApproach, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.5f);
        
        // Step 2: 下降到 Pick 位置（抓取）
        Vector3 pickPos = rootPos + pickPosition;
        LogMessage($"[Test7] Step 2/6: Move to pick position - {pickPos}");
        yield return StartCoroutine(MoveToCartesianPosition(pickPos, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.3f);
        LogMessage("[Test7] (Gripper close - simulated)");
        
        // Step 3: 抬起（提升）
        Vector3 pickLift = rootPos + pickPosition + Vector3.up * liftHeight;
        LogMessage($"[Test7] Step 3/6: Lift from pick - {pickLift}");
        yield return StartCoroutine(MoveToCartesianPosition(pickLift, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.5f);
        
        // Step 4: 移动到 Place 位置上方
        Vector3 placeApproach = rootPos + placePosition + Vector3.up * liftHeight;
        LogMessage($"[Test7] Step 4/6: Approach place position - {placeApproach}");
        yield return StartCoroutine(MoveToCartesianPosition(placeApproach, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.5f);
        
        // Step 5: 下降到 Place 位置（放置）
        Vector3 placePos = rootPos + placePosition;
        LogMessage($"[Test7] Step 5/6: Move to place position - {placePos}");
        yield return StartCoroutine(MoveToCartesianPosition(placePos, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.3f);
        LogMessage("[Test7] (Gripper open - simulated)");
        
        // Step 6: 抬起离开
        Vector3 placeLift = rootPos + placePosition + Vector3.up * liftHeight;
        LogMessage($"[Test7] Step 6/6: Lift from place - {placeLift}");
        yield return StartCoroutine(MoveToCartesianPosition(placeLift, downRotation));
        if (lastMoveSuccess) successCount++;
        yield return new WaitForSeconds(pauseBetweenTests * 0.5f);
        
        bool passed = successCount >= totalSteps * 0.8f; // 80% 成功率
        LogMessage($"[Test7] Success: {successCount}/{totalSteps} - {(passed ? "PASS" : "FAIL")}");
        
        if (passed) passedTests++;
    }
    
    #endregion
    
    #region 运动控制
    
    /// <summary>
    /// 移动到指定关节位置
    /// </summary>
    IEnumerator MoveToJointPosition(Vector<float> targetJoints)
    {
        if (useSmoothMotion)
        {
            // 平滑运动：使用增量驱动
            float timeout = 30f; // 最大等待时间
            float startTime = Time.time;
            
            while (Time.time - startTime < timeout)
            {
                // 每个 FixedUpdate 调用一次增量驱动
                bool reached = robot.DriveJointsIncremental(targetJoints, jointSpeed);
                
                if (reached)
                {
                    // 到达目标，等待稳定
                    yield return new WaitForSeconds(0.1f);
                    break;
                }
                
                // 等待下一个物理帧
                yield return new WaitForFixedUpdate();
            }
        }
        else
        {
            // 瞬移模式：直接设置目标
            robot.DriveJoints(targetJoints);
            
            // 等待到达（简单的时间估计）
            float maxAngleChange = 0f;
            Vector<float> currentJoints = robot.GetCurrentJointState();
            for (int i = 0; i < targetJoints.Count; i++)
            {
                float diff = Mathf.Abs(targetJoints[i] - currentJoints[i]);
                if (diff > maxAngleChange) maxAngleChange = diff;
            }
            
            float estimatedTime = maxAngleChange / jointSpeed + 0.5f;
            yield return new WaitForSeconds(estimatedTime);
        }
    }
    
    /// <summary>
    /// 移动到指定笛卡尔位置
    /// </summary>
    IEnumerator MoveToCartesianPosition(Vector3 position, Quaternion rotation)
    {
        // 更新可视化目标
        currentTargetPosition = position;
        currentTargetRotation = rotation;
        hasActiveTarget = true;
        
        // 检查工作空间
        if (!robot.InsideWorkspace(position))
        {
            LogMessage($"  [IK] Position {position} outside workspace - SKIP");
            lastMoveSuccess = false;
            hasActiveTarget = false;
            yield break;
        }
        
        // 创建目标位姿
        Matrix4x4 targetPose = Matrix4x4.TRS(position, rotation, Vector3.one);
        
        // 求解 IK
        float solveStartTime = Time.realtimeSinceStartup;
        
        robot.SyncJointState();
        Vector<float> initialGuess = robot.GetCurrentJointState();
        
        Vector<float> solution = robot.SolveInverseKinematics(targetPose, initialGuess);
        
        float solveTime = Time.realtimeSinceStartup - solveStartTime;
        totalSolveTime += solveTime;
        
        // 检查求解结果
        Vector<float> posError = robot.IKSolver.p_err;
        float positionError = Mathf.Sqrt(posError[0] * posError[0] + posError[1] * posError[1] + posError[2] * posError[2]);
        float orientationError = Mathf.Sqrt(posError[3] * posError[3] + posError[4] * posError[4] + posError[5] * posError[5]);
        
        bool ikSuccess = positionError < 0.01f; // 10mm 误差
        
        LogMessage($"  [IK] Solve time: {solveTime * 1000:F1}ms, Pos err: {positionError * 1000:F1}mm, Ori err: {orientationError:F3} - {(ikSuccess ? "OK" : "FAIL")}");
        
        if (!ikSuccess)
        {
            LogMessage($"  [IK] Solution joints: [{FormatVector(solution)}]");
            lastMoveSuccess = false;
            hasActiveTarget = false;
            yield break;
        }
        
        // 执行运动
        yield return MoveToJointPosition(solution);
        
        // 验证最终位置
        robot.SyncJointState();
        Vector3 finalPos = robot.GetCurrentEndEffectorPosition();
        float finalError = Vector3.Distance(finalPos, position);
        
        LogMessage($"  [Move] Final pos: {finalPos}, Error: {finalError * 1000:F1}mm");
        
        lastMoveSuccess = finalError < 0.02f; // 20mm 容忍（包括物理仿真误差）
        hasActiveTarget = false;
    }
    
    #endregion
    
    #region 实体 Pick & Place
    
    /// <summary>
    /// 启动实体 Pick & Place 测试
    /// </summary>
    void StartPhysicalPickPlace()
    {
        if (isTestRunning)
        {
            LogMessage("[Warning] Test already running, please wait...");
            return;
        }
        
        // 确保有 PickPlaceManager
        EnsurePickPlaceManager();
        
        if (pickPlaceManager == null)
        {
            LogMessage("[Error] Failed to create PickPlaceManager!");
            return;
        }
        
        LogMessage("");
        LogMessage("========== Starting Physical Pick & Place ==========");
        
        // 同步配置
        pickPlaceManager.pickPosition = pickPosition;
        pickPlaceManager.placePosition = placePosition;
        pickPlaceManager.approachHeight = liftHeight;
        pickPlaceManager.downwardOrientation = downwardOrientation;
        pickPlaceManager.jointSpeed = jointSpeed;
        
        // 开始测试
        pickPlaceManager.StartTest();
    }
    
    /// <summary>
    /// 重置实体 Pick & Place 测试
    /// </summary>
    void ResetPhysicalPickPlace()
    {
        if (pickPlaceManager != null)
        {
            pickPlaceManager.ResetTest();
            LogMessage("[Physical P&P] Reset");
        }
    }
    
    /// <summary>
    /// 确保 PickPlaceManager 存在
    /// </summary>
    void EnsurePickPlaceManager()
    {
        if (pickPlaceManager != null) return;
        
        // 尝试查找现有的
        pickPlaceManager = FindObjectOfType<PickPlaceManager>();
        
        if (pickPlaceManager == null)
        {
            // 创建新的
            GameObject managerObj = new GameObject("PickPlaceManager");
            pickPlaceManager = managerObj.AddComponent<PickPlaceManager>();
            pickPlaceManager.ikTester = this;
            
            // 尝试查找 PincherController
            pickPlaceManager.pincherController = FindObjectOfType<PincherController>();
            
            LogMessage("[Init] Created PickPlaceManager");
        }
        
        // 确保有 GripperGraspController
        EnsureGraspController();
        
        // 设置抓取控制器
        if (graspController != null)
        {
            pickPlaceManager.graspController = graspController;
        }
    }
    
    /// <summary>
    /// 确保 GripperGraspController 存在
    /// </summary>
    void EnsureGraspController()
    {
        if (graspController != null) return;
        
        // 尝试查找现有的
        graspController = FindObjectOfType<GripperGraspController>();
        
        if (graspController == null)
        {
            // 尝试在 HandE 上创建
            PincherController pincher = FindObjectOfType<PincherController>();
            if (pincher != null)
            {
                graspController = pincher.gameObject.AddComponent<GripperGraspController>();
                graspController.pincherController = pincher;
                
                // 查找 TCP 作为连接点
                Transform tcp = pincher.transform.Find("TCP");
                if (tcp != null)
                {
                    graspController.graspAttachPoint = tcp;
                }
                
                LogMessage("[Init] Created GripperGraspController on HandE");
            }
            else
            {
                LogMessage("[Warning] Cannot create GripperGraspController - PincherController not found");
            }
        }
    }
    
    #endregion
    
    #region 辅助方法
    
    void ResetRobot()
    {
        if (isTestRunning && currentTestCoroutine != null)
        {
            StopCoroutine(currentTestCoroutine);
            isTestRunning = false;
        }
        
        LogMessage("[Reset] Moving to Home position...");
        robot.MoveToHome();
    }
    
    void PrintCurrentState()
    {
        robot.SyncJointState();
        
        LogMessage("");
        LogMessage("--- Current State ---");
        LogMessage($"Joints (deg): [{FormatVector(robot.GetCurrentJointState())}]");
        LogMessage($"EE Position: {robot.GetCurrentEndEffectorPosition()}");
        LogMessage($"EE Rotation: {robot.GetCurrentEndEffectorRotation().eulerAngles}");
        LogMessage($"Tests: {passedTests}/{totalTests} passed");
        LogMessage($"Avg solve time: {(totalTests > 0 ? totalSolveTime / totalTests * 1000 : 0):F1}ms");
        LogMessage("--------------------");
        LogMessage("");
    }
    
    void SaveLogToFile()
    {
        try
        {
            // 确保目录存在
            string directory = Path.GetDirectoryName(logFilePath);
            if (!Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            
            // 添加摘要
            logBuilder.AppendLine("");
            logBuilder.AppendLine("========================================");
            logBuilder.AppendLine("TEST SUMMARY");
            logBuilder.AppendLine("========================================");
            logBuilder.AppendLine($"Total tests: {totalTests}");
            logBuilder.AppendLine($"Passed: {passedTests}");
            logBuilder.AppendLine($"Failed: {totalTests - passedTests}");
            logBuilder.AppendLine($"Pass rate: {(totalTests > 0 ? (float)passedTests / totalTests * 100 : 0):F1}%");
            logBuilder.AppendLine($"Average solve time: {(totalTests > 0 ? totalSolveTime / totalTests * 1000 : 0):F1}ms");
            logBuilder.AppendLine($"Log saved: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            
            File.WriteAllText(logFilePath, logBuilder.ToString());
            
            Debug.Log($"[UR3eIKTester] Log saved to: {logFilePath}");
            LogMessage($"[Log] Saved to: {logFilePath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[UR3eIKTester] Failed to save log: {e.Message}");
        }
    }
    
    void LogMessage(string message)
    {
        string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
        string formattedMessage = $"[{timestamp}] {message}";
        
        logBuilder.AppendLine(formattedMessage);
        
        if (verboseConsole)
        {
            Debug.Log(formattedMessage);
        }
    }
    
    string FormatVector(Vector<float> vec)
    {
        if (vec == null) return "null";
        
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < vec.Count; i++)
        {
            sb.Append($"{vec[i]:F1}");
            if (i < vec.Count - 1) sb.Append(", ");
        }
        return sb.ToString();
    }
    
    #endregion
    
    #region FK 验证与轴向调试
    
    /// <summary>
    /// FK验证：比较Chain计算的EE位置与Unity实际TCP位置
    /// </summary>
    void RunFKValidation()
    {
        LogMessage("");
        LogMessage("========================================");
        LogMessage("FK VALIDATION - Comparing Chain FK vs Unity Transform");
        LogMessage("========================================");
        
        // 1. 当前姿态下的FK验证
        ValidateFKAtCurrentPose();
        
        // 2. Home位置验证
        LogMessage("");
        LogMessage("--- Test at Home Position ---");
        robot.MoveToHome();
        robot.SyncJointState();
        ValidateFKAtCurrentPose();
        
        // 3. 各关节单独运动测试
        LogMessage("");
        LogMessage("--- Single Joint Motion Tests ---");
        for (int jointIdx = 0; jointIdx < 6; jointIdx++)
        {
            TestSingleJointFK(jointIdx);
        }
        
        // 4. 输出Chain内部状态
        LogMessage("");
        LogMessage("--- Chain Internal State ---");
        PrintChainState();
        
        LogMessage("========================================");
        LogMessage("FK VALIDATION COMPLETE");
        LogMessage("========================================");
        
        // 自动保存log
        SaveLogToFile();
    }
    
    void ValidateFKAtCurrentPose()
    {
        robot.SyncJointState();
        
        // 获取当前关节角度
        Vector<float> jointAngles = robot.GetCurrentJointState();
        LogMessage($"Joint angles (deg): [{FormatVector(jointAngles)}]");
        
        // 获取Chain计算的EE位置 (从base2EETransform)
        Matrix4x4 chainFK = robot.Articulation.base2EETransform;
        Vector3 chainPos = chainFK.GetColumn(3); // 提取平移部分
        Quaternion chainRot = chainFK.rotation;
        
        LogMessage($"Chain FK position: ({chainPos.x:F4}, {chainPos.y:F4}, {chainPos.z:F4})");
        LogMessage($"Chain FK rotation: {chainRot.eulerAngles}");
        
        // 获取Unity实际TCP位置
        if (tcpTransform != null)
        {
            Vector3 unityPos = tcpTransform.position;
            Quaternion unityRot = tcpTransform.rotation;
            
            // 需要转换到base坐标系 (使用第一个segment的transform作为base)
            Transform baseTransform = robot.Articulation.segments[0].linkedBody.transform;
            Vector3 unityPosLocal = baseTransform.InverseTransformPoint(unityPos);
            Quaternion unityRotLocal = Quaternion.Inverse(baseTransform.rotation) * unityRot;
            
            LogMessage($"Unity TCP position (world): ({unityPos.x:F4}, {unityPos.y:F4}, {unityPos.z:F4})");
            LogMessage($"Unity TCP position (base-local): ({unityPosLocal.x:F4}, {unityPosLocal.y:F4}, {unityPosLocal.z:F4})");
            LogMessage($"Unity TCP rotation (base-local): {unityRotLocal.eulerAngles}");
            
            // 计算误差
            float posError = Vector3.Distance(chainPos, unityPosLocal);
            float rotError = Quaternion.Angle(chainRot, unityRotLocal);
            
            LogMessage($">>> Position Error: {posError * 1000:F2} mm");
            LogMessage($">>> Rotation Error: {rotError:F2} deg");
            
            if (posError > 0.01f) // 10mm
            {
                LogMessage("!!! WARNING: Large FK position error detected !!!");
            }
        }
        else
        {
            LogMessage("WARNING: tcpTransform not assigned, skipping Unity comparison");
            
            // 至少打印Chain内部的一致性检查
            Vector3 eeFromRobot = robot.GetCurrentEndEffectorPosition();
            LogMessage($"UR3eRobot.GetCurrentEndEffectorPosition(): ({eeFromRobot.x:F4}, {eeFromRobot.y:F4}, {eeFromRobot.z:F4})");
            
            float internalDiff = Vector3.Distance(chainPos, eeFromRobot);
            LogMessage($"Internal consistency (Chain vs Robot): {internalDiff * 1000:F2} mm");
        }
    }
    
    void TestSingleJointFK(int jointIndex)
    {
        LogMessage($"");
        LogMessage($"Testing Joint {jointIndex}:");
        
        // 使用 Home 关节角度（度数）: [0, -90, 0, -90, 0, 0]
        float[] homeJoints = { 0, -90, 0, -90, 0, 0 };
        Vector<float> homeJointsVec = Vector<float>.Build.DenseOfArray(homeJoints);
        
        // 直接更新运动学（不依赖物理引擎）
        robot.Articulation.UpdateChainKinematics(homeJointsVec);
        robot.Articulation.JntToCart();
        
        // 记录Home位置
        Matrix4x4 homeFK = robot.Articulation.base2EETransform;
        Vector3 homePos = homeFK.GetColumn(3);
        LogMessage($"  Home FK pos: ({homePos.x:F4}, {homePos.y:F4}, {homePos.z:F4})");
        
        // 移动单个关节 +30度
        float[] testJoints = (float[])homeJoints.Clone();
        testJoints[jointIndex] += 30f;
        Vector<float> testJointsVec = Vector<float>.Build.DenseOfArray(testJoints);
        
        // 直接更新运动学计算（绕过物理引擎）
        robot.Articulation.UpdateChainKinematics(testJointsVec);
        robot.Articulation.JntToCart();
        
        // 获取新的FK
        Matrix4x4 newFK = robot.Articulation.base2EETransform;
        Vector3 newPos = newFK.GetColumn(3);
        
        Vector3 deltaPos = newPos - homePos;
        float deltaMag = deltaPos.magnitude;
        
        LogMessage($"  After +30deg: FK pos: ({newPos.x:F4}, {newPos.y:F4}, {newPos.z:F4})");
        LogMessage($"  Delta: ({deltaPos.x:F4}, {deltaPos.y:F4}, {deltaPos.z:F4}), mag: {deltaMag * 1000:F1}mm");
        
        // 检查是否有响应
        if (deltaMag < 0.001f && jointIndex < 5)  // Joint 5 (wrist roll) 可能位移很小
        {
            LogMessage($"  !!! WARNING: No FK response to joint motion !!!");
        }
        
        // 判断是否合理（base关节应该主要影响XZ平面）
        string expectedEffect = jointIndex switch
        {
            0 => "XZ rotation (around Y)",
            1 => "Y elevation change",
            2 => "Y/Z (elbow)",
            3 => "Wrist pitch",
            4 => "Wrist yaw",
            5 => "Wrist roll (minimal pos change)",
            _ => "Unknown"
        };
        LogMessage($"  Expected effect: {expectedEffect}");
    }
    
    /// <summary>
    /// 轴向调试：打印每个Segment的配置和ArticulationBody锚点信息
    /// </summary>
    void RunAxisDebug()
    {
        LogMessage("");
        LogMessage("========================================");
        LogMessage("AXIS DEBUG - Segment & ArticulationBody Info");
        LogMessage("========================================");
        
        Chain chain = robot.Articulation;
        
        LogMessage($"Chain has {chain.segments.Count} segments");
        LogMessage($"Chain has {chain.numberJoints} active joints");
        LogMessage("");
        
        for (int i = 0; i < chain.segments.Count; i++)
        {
            Segment seg = chain.segments[i];
            ArticulationBody ab = seg.linkedBody;
            
            LogMessage($"--- Segment [{i}]: {ab.name} ---");
            LogMessage($"  index: {seg.index}");
            LogMessage($"  jointIndex (axis): {seg.jointIndex}");
            LogMessage($"  jointWorldIndex: {seg.jointWorldIndex}");
            LogMessage($"  localPosition: {seg.localPosition}");
            LogMessage($"  localRotation: {seg.localRotation.eulerAngles}");
            LogMessage($"  localJointState: {seg.localJointState:F4}");
            
            // IKJoint 信息
            if (seg.joint != null)
            {
                LogMessage($"  IKJoint:");
                LogMessage($"    rotationState: {seg.joint.rotationState}");
            }
            
            // ArticulationBody信息
            if (ab != null)
            {
                LogMessage($"  ArticulationBody:");
                LogMessage($"    jointType: {ab.jointType}");
                LogMessage($"    anchorPosition: {ab.anchorPosition}");
                LogMessage($"    anchorRotation: {ab.anchorRotation.eulerAngles}");
                LogMessage($"    parentAnchorPosition: {ab.parentAnchorPosition}");
                LogMessage($"    parentAnchorRotation: {ab.parentAnchorRotation.eulerAngles}");
                
                // 计算实际旋转轴（在父坐标系中）
                Vector3 localAxis = ab.anchorRotation * Vector3.right; // X轴是默认revolute轴
                LogMessage($"    Computed local axis: {localAxis}");
                
                if (ab.jointType == ArticulationJointType.RevoluteJoint)
                {
                    LogMessage($"    xDrive: lower={ab.xDrive.lowerLimit:F1}, upper={ab.xDrive.upperLimit:F1}");
                }
            }
            
            // 实际 Unity Transform
            LogMessage($"  Unity Transform (local): pos={ab.transform.localPosition}, rot={ab.transform.localRotation.eulerAngles}");
            LogMessage("");
        }
        
        // 打印base2EETransform分解
        LogMessage("--- base2EETransform Decomposition ---");
        Matrix4x4 m = chain.base2EETransform;
        LogMessage($"Position: ({m.m03:F4}, {m.m13:F4}, {m.m23:F4})");
        LogMessage($"Rotation matrix:");
        LogMessage($"  [{m.m00:F4}, {m.m01:F4}, {m.m02:F4}]");
        LogMessage($"  [{m.m10:F4}, {m.m11:F4}, {m.m12:F4}]");
        LogMessage($"  [{m.m20:F4}, {m.m21:F4}, {m.m22:F4}]");
        
        LogMessage("");
        LogMessage("========================================");
        LogMessage("AXIS DEBUG COMPLETE");
        LogMessage("========================================");
        
        // 自动保存log
        SaveLogToFile();
    }
    
    void PrintChainState()
    {
        Chain chain = robot.Articulation;
        
        LogMessage($"base2EETransform:");
        Matrix4x4 m = chain.base2EETransform;
        LogMessage($"  Position: ({m.m03:F4}, {m.m13:F4}, {m.m23:F4})");
        
        // 打印每个segment的变换
        LogMessage($"Segment transforms (local):");
        for (int i = 0; i < chain.segments.Count; i++)
        {
            Segment seg = chain.segments[i];
            ArticulationBody ab = seg.linkedBody;
            Vector3 pos = ab.transform.localPosition;
            Vector3 rot = ab.transform.localRotation.eulerAngles;
            LogMessage($"  [{i}] {ab.name}: pos=({pos.x:F4}, {pos.y:F4}, {pos.z:F4}), rot=({rot.x:F1}, {rot.y:F1}, {rot.z:F1})");
        }
    }
    
    #endregion
    
    #region GUI 显示
    
    void OnGUI()
    {
        if (robot == null) return;
        
        // 创建样式
        GUIStyle boxStyle = new GUIStyle(GUI.skin.box);
        boxStyle.normal.background = MakeTexture(2, 2, new Color(0f, 0f, 0f, 0.7f));
        
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 14;
        labelStyle.normal.textColor = Color.white;
        
        GUIStyle headerStyle = new GUIStyle(labelStyle);
        headerStyle.fontStyle = FontStyle.Bold;
        headerStyle.fontSize = 16;
        
        // 控制面板
        GUILayout.BeginArea(new Rect(10, 10, 280, 380), boxStyle);
        GUILayout.Label("UR3e IK Tester (Copies/)", headerStyle);
        GUILayout.Space(10);
        
        GUILayout.Label("=== Tests ===", GUI.skin.box);
        GUILayout.Label("[1] Home Position", labelStyle);
        GUILayout.Label("[2] Forward Reach", labelStyle);
        GUILayout.Label("[3] Square Trajectory", labelStyle);
        GUILayout.Label("[4] Random Points", labelStyle);
        GUILayout.Label("[7] Pick and Place", labelStyle);
        GUILayout.Space(5);
        
        GUILayout.Label("=== Debug ===", GUI.skin.box);
        GUILayout.Label("[5] FK Validation", labelStyle);
        GUILayout.Label("[6] Axis Debug", labelStyle);
        GUILayout.Space(5);
        
        GUILayout.Label("=== Controls ===", GUI.skin.box);
        GUILayout.Label("[R] Reset", labelStyle);
        GUILayout.Label("[P] Print State", labelStyle);
        GUILayout.Label("[L] Save Log", labelStyle);
        GUILayout.Space(10);
        
        // 状态显示
        string statusText = isTestRunning ? "🔄 Running..." : "⏸ Idle";
        GUILayout.Label($"Status: {statusText}", labelStyle);
        GUILayout.Label($"Tests: {passedTests}/{totalTests} passed", labelStyle);
        
        GUILayout.EndArea();
        
        // 状态面板
        GUILayout.BeginArea(new Rect(Screen.width - 320, 10, 310, 180), boxStyle);
        GUILayout.Label("Current State", headerStyle);
        GUILayout.Space(5);
        
        Vector3 eePos = robot.GetCurrentEndEffectorPosition();
        GUILayout.Label($"EE Pos: ({eePos.x:F3}, {eePos.y:F3}, {eePos.z:F3})", labelStyle);
        
        Vector3 eeRot = robot.GetCurrentEndEffectorRotation().eulerAngles;
        GUILayout.Label($"EE Rot: ({eeRot.x:F1}, {eeRot.y:F1}, {eeRot.z:F1})", labelStyle);
        
        GUILayout.Space(5);
        
        if (robot.IKSolver.p_err != null)
        {
            float posErr = Mathf.Sqrt(
                robot.IKSolver.p_err[0] * robot.IKSolver.p_err[0] +
                robot.IKSolver.p_err[1] * robot.IKSolver.p_err[1] +
                robot.IKSolver.p_err[2] * robot.IKSolver.p_err[2]
            );
            GUILayout.Label($"Last IK Pos Error: {posErr * 1000:F2}mm", labelStyle);
        }
        
        float avgTime = totalTests > 0 ? totalSolveTime / totalTests * 1000 : 0;
        GUILayout.Label($"Avg Solve Time: {avgTime:F1}ms", labelStyle);
        
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
    
    #region 可视化
    
    void OnDrawGizmos()
    {
        if (!showTargetVisualization || !hasActiveTarget)
            return;
            
        // 绘制目标位置球体
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(currentTargetPosition, targetSphereSize);
        Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
        Gizmos.DrawSphere(currentTargetPosition, targetSphereSize);
        
        // 绘制目标方向坐标轴
        Vector3 pos = currentTargetPosition;
        
        // X 轴 - 红色
        Gizmos.color = Color.red;
        Vector3 xAxis = currentTargetRotation * Vector3.right * targetAxisLength;
        Gizmos.DrawLine(pos, pos + xAxis);
        DrawArrowHead(pos + xAxis, xAxis.normalized, Color.red);
        
        // Y 轴 - 绿色
        Gizmos.color = Color.green;
        Vector3 yAxis = currentTargetRotation * Vector3.up * targetAxisLength;
        Gizmos.DrawLine(pos, pos + yAxis);
        DrawArrowHead(pos + yAxis, yAxis.normalized, Color.green);
        
        // Z 轴 - 蓝色
        Gizmos.color = Color.blue;
        Vector3 zAxis = currentTargetRotation * Vector3.forward * targetAxisLength;
        Gizmos.DrawLine(pos, pos + zAxis);
        DrawArrowHead(pos + zAxis, zAxis.normalized, Color.blue);
        
        // 绘制到机器人基座的参考线
        if (robot != null && robot.Articulation != null && robot.Articulation.segments.Count > 0)
        {
            Gizmos.color = new Color(1f, 1f, 0f, 0.2f);
            Vector3 basePos = robot.Articulation.segments[0].linkedBody.transform.position;
            Gizmos.DrawLine(basePos, currentTargetPosition);
        }
    }
    
    void DrawArrowHead(Vector3 tip, Vector3 direction, Color color)
    {
        Gizmos.color = color;
        float arrowSize = targetAxisLength * 0.15f;
        
        // 创建垂直于方向的两个向量
        Vector3 right = Vector3.Cross(direction, Vector3.up).normalized;
        if (right.magnitude < 0.01f)
            right = Vector3.Cross(direction, Vector3.right).normalized;
        Vector3 up = Vector3.Cross(right, direction).normalized;
        
        // 绘制箭头
        Vector3 backBase = tip - direction * arrowSize;
        Gizmos.DrawLine(tip, backBase + right * arrowSize * 0.5f);
        Gizmos.DrawLine(tip, backBase - right * arrowSize * 0.5f);
        Gizmos.DrawLine(tip, backBase + up * arrowSize * 0.5f);
        Gizmos.DrawLine(tip, backBase - up * arrowSize * 0.5f);
    }
    
    #endregion
}
