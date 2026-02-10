using System;
using System.Collections;
using System.IO;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/// <summary>
/// TCP/Tool Offset 调试脚本
/// 
/// 用于诊断 TCP 位置偏移问题（设置Y=0但实际到达Y<0）
/// 以及 Orientation 约定问题（为什么需要 -90,0,0 而不是 180,0,0）
/// 
/// 键盘控制：
/// - [D] 运行完整调试（步骤1-3）
/// - [T] 测试单个目标点并验证偏移
/// - [O] Orientation 对比测试
/// - [L] 保存日志到文件
/// </summary>
public class ToolOffsetDebugger : MonoBehaviour
{
    #region Inspector 配置
    
    [Header("Robot References")]
    [Tooltip("UR3e 根对象")]
    public GameObject robotRoot;
    
    [Tooltip("手动指定的 TCP Transform（如果有）")]
    public Transform tcpTransform;
    
    [Tooltip("末端执行器（HandE）Transform")]
    public Transform eeTransform;
    
    [Header("Test Settings")]
    [Tooltip("测试目标位置（相对于基座）")]
    public Vector3 testTargetRelative = new Vector3(0.3f, 0.0f, 0.0f);
    
    [Tooltip("测试目标朝向（Euler角）")]
    public Vector3 testOrientation = new Vector3(-90f, 0f, 0f);
    
    [Header("Debug Output")]
    [Tooltip("日志输出路径")]
    public string logFilePath = "Assets/Scripts/Copies/Debug/ToolOffset_Debug.txt";
    
    #endregion
    
    #region 私有变量
    
    private UR3eRobot robot;
    private StringBuilder logBuilder = new StringBuilder();
    private bool isInitialized = false;
    
    #endregion
    
    #region Unity 生命周期
    
    void Start()
    {
        Initialize();
    }
    
    void Update()
    {
        HandleInput();
    }
    
    #endregion
    
    #region 初始化
    
    void Initialize()
    {
        if (robotRoot == null)
        {
            Debug.LogError("[ToolOffsetDebugger] robotRoot not assigned!");
            return;
        }
        
        try
        {
            robot = new UR3eRobot(robotRoot);
            isInitialized = true;
            
            Log("========================================");
            Log("Tool Offset Debugger Initialized");
            Log($"Date: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Log("========================================");
            Log("");
            Log("Keyboard Controls:");
            Log("  [D] Run full debug (Step 1-3)");
            Log("  [T] Test single target point");
            Log("  [O] Orientation comparison test");
            Log("  [8] Isolate test (disable toolVector)");
            Log("  [L] Save log to file");
            Log("");
        }
        catch (Exception e)
        {
            Debug.LogError($"[ToolOffsetDebugger] Init failed: {e.Message}");
        }
    }
    
    void HandleInput()
    {
        if (!isInitialized) return;
        
        if (Input.GetKeyDown(KeyCode.D))
        {
            RunFullDebug();
        }
        else if (Input.GetKeyDown(KeyCode.T))
        {
            StartCoroutine(TestSingleTargetWithOffset());
        }
        else if (Input.GetKeyDown(KeyCode.O))
        {
            RunOrientationComparison();
        }
        else if (Input.GetKeyDown(KeyCode.L))
        {
            SaveLogToFile();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha8))
        {
            StartCoroutine(TestWithoutToolVector());
        }
    }
    
    #endregion
    
    #region 步骤1: 验证Tool Vector的正确性
    
    void RunStep1_ValidateToolVector()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  步骤1: 验证 Tool Vector 的正确性                              ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        Chain chain = robot.Articulation;
        GripperTool gripper = chain.Gripper;
        
        // 1.1 打印 GripperTool 中存储的 toolVector
        Log("[1.1] GripperTool.toolVector (从Chain自动检测):");
        Log($"     toolVector = {gripper.toolVector}");
        Log($"     toolVector magnitude = {gripper.toolVector.magnitude:F4} m");
        Log("");
        
        // 1.2 如果有手动指定的 TCP Transform，计算期望偏移
        if (tcpTransform != null && eeTransform != null)
        {
            Log("[1.2] 手动验证 EE->TCP 偏移:");
            Log($"     EE (HandE) World Position: {eeTransform.position}");
            Log($"     EE (HandE) World Rotation: {eeTransform.rotation.eulerAngles}");
            Log($"     TCP World Position: {tcpTransform.position}");
            Log($"     TCP World Rotation: {tcpTransform.rotation.eulerAngles}");
            
            // 计算局部偏移 (EE坐标系下TCP的位置)
            Vector3 tcpInEELocal = eeTransform.InverseTransformPoint(tcpTransform.position);
            Quaternion tcpRotInEELocal = Quaternion.Inverse(eeTransform.rotation) * tcpTransform.rotation;
            
            Log("");
            Log($"     >>> TCP in EE Local Space:");
            Log($"         Position: {tcpInEELocal}");
            Log($"         Rotation: {tcpRotInEELocal.eulerAngles}");
            
            // 与 GripperTool.toolVector 对比
            Vector3 diff = tcpInEELocal - gripper.toolVector;
            Log("");
            Log($"     >>> 与 GripperTool.toolVector 的差异:");
            Log($"         Delta: {diff}");
            Log($"         Delta magnitude: {diff.magnitude * 1000:F2} mm");
            
            if (diff.magnitude > 0.001f)
            {
                Log("     !!! WARNING: Tool vector 与实际 EE->TCP 偏移不一致 !!!");
            }
            else
            {
                Log("     ✓ Tool vector 与实际偏移一致");
            }
        }
        else
        {
            Log("[1.2] TCP Transform 或 EE Transform 未指定，跳过手动验证");
            Log("      请在Inspector中拖入 tcpTransform 和 eeTransform");
        }
        
        // 1.3 检查 eeBody 是否正确
        Log("");
        Log("[1.3] Chain 末端执行器信息:");
        if (chain.eeBody != null)
        {
            Log($"     eeBody name: {chain.eeBody.name}");
            Log($"     eeBody position (world): {chain.eeBody.transform.position}");
            Log($"     eeBody rotation (world): {chain.eeBody.transform.rotation.eulerAngles}");
        }
        else
        {
            Log("     !!! eeBody is NULL !!!");
        }
        
        // 1.4 检查 toolVector 在哪个方向
        Log("");
        Log("[1.4] toolVector 方向分析:");
        Vector3 tv = gripper.toolVector;
        if (Mathf.Abs(tv.x) > Mathf.Abs(tv.y) && Mathf.Abs(tv.x) > Mathf.Abs(tv.z))
            Log($"     主要沿 X 轴方向");
        else if (Mathf.Abs(tv.y) > Mathf.Abs(tv.x) && Mathf.Abs(tv.y) > Mathf.Abs(tv.z))
            Log($"     主要沿 Y 轴方向");
        else if (Mathf.Abs(tv.z) > Mathf.Abs(tv.x) && Mathf.Abs(tv.z) > Mathf.Abs(tv.y))
            Log($"     主要沿 Z 轴方向（通常是工具指向方向）");
        else
            Log($"     无明显主方向（可能接近零或对角）");
    }
    
    #endregion
    
    #region 步骤2: 验证FK输出点位
    
    void RunStep2_ValidateFKOutput()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  步骤2: 验证 FK 输出点位                                       ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        Chain chain = robot.Articulation;
        
        // 同步当前关节状态
        robot.SyncJointState();
        Vector<float> currentJoints = robot.GetCurrentJointState();
        
        Log("[2.1] 当前关节角度 (度):");
        Log($"     [{FormatVector(currentJoints)}]");
        Log("");
        
        // 2.2 获取 base2EETransform (FK计算结果)
        Matrix4x4 fkResult = chain.base2EETransform;
        Vector3 fkPosition = new Vector3(fkResult.m03, fkResult.m13, fkResult.m23);
        Quaternion fkRotation = fkResult.rotation;
        
        Log("[2.2] Chain FK 计算结果 (base2EETransform):");
        Log($"     FK Position: {fkPosition}");
        Log($"     FK Position (mm): ({fkPosition.x * 1000:F1}, {fkPosition.y * 1000:F1}, {fkPosition.z * 1000:F1})");
        Log($"     FK Rotation (Euler): {fkRotation.eulerAngles}");
        Log("");
        
        // 2.3 获取 Unity 中实际 TCP 位置
        if (tcpTransform != null)
        {
            Vector3 tcpWorldPos = tcpTransform.position;
            Quaternion tcpWorldRot = tcpTransform.rotation;
            
            Log("[2.3] Unity TCP Transform (世界坐标):");
            Log($"     TCP World Position: {tcpWorldPos}");
            Log($"     TCP World Rotation: {tcpWorldRot.eulerAngles}");
            
            // 转换到机器人基座坐标系
            Transform baseTransform = chain.segments[0].linkedBody.transform.parent; // 获取base的parent（通常是机器人根）
            Vector3 basePos = chain.rootPosition;
            Quaternion baseRot = chain.rootRotation;
            
            // TCP在基座坐标系中的位置
            Vector3 tcpInBase = Quaternion.Inverse(baseRot) * (tcpWorldPos - basePos);
            Quaternion tcpRotInBase = Quaternion.Inverse(baseRot) * tcpWorldRot;
            
            Log("");
            Log("[2.4] TCP 在基座坐标系中的位置:");
            Log($"     Base Position: {basePos}");
            Log($"     Base Rotation: {baseRot.eulerAngles}");
            Log($"     TCP in Base: {tcpInBase}");
            Log($"     TCP in Base (mm): ({tcpInBase.x * 1000:F1}, {tcpInBase.y * 1000:F1}, {tcpInBase.z * 1000:F1})");
            Log($"     TCP Rot in Base: {tcpRotInBase.eulerAngles}");
            
            // 计算差值
            Vector3 positionDiff = fkPosition - tcpInBase;
            float rotationDiff = Quaternion.Angle(fkRotation, tcpRotInBase);
            
            Log("");
            Log("[2.5] FK vs Unity TCP 差值:");
            Log($"     Position Difference: {positionDiff}");
            Log($"     Position Diff (mm): ({positionDiff.x * 1000:F2}, {positionDiff.y * 1000:F2}, {positionDiff.z * 1000:F2})");
            Log($"     Position Diff Magnitude: {positionDiff.magnitude * 1000:F2} mm");
            Log($"     Rotation Difference: {rotationDiff:F2} degrees");
            
            if (positionDiff.magnitude > 0.005f)
            {
                Log("");
                Log("     !!! WARNING: FK计算位置与Unity TCP位置差异超过5mm !!!");
                Log("     可能原因:");
                Log("     - toolVector 未正确应用");
                Log("     - 链条遍历未正确停止在EE");
                Log("     - 存在未计入的局部偏移");
            }
        }
        else
        {
            Log("[2.3] TCP Transform 未指定，无法与Unity比较");
        }
        
        // 2.6 打印 segmentPositions（FK中间结果）
        Log("");
        Log("[2.6] FK 中间结果 - segmentPositions:");
        for (int i = 0; i < chain.segmentPositions.Length; i++)
        {
            Vector3 pos = chain.segmentPositions[i];
            string segName = i < chain.segments.Count ? chain.segments[i].linkedBody.name : "N/A";
            Log($"     [{i}] {segName}: ({pos.x:F4}, {pos.y:F4}, {pos.z:F4})");
        }
    }
    
    #endregion
    
    #region 步骤3: 确认链条终点
    
    void RunStep3_ValidateChainTermination()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  步骤3: 确认链条终点                                           ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        Chain chain = robot.Articulation;
        
        // 3.1 链的基本信息
        Log("[3.1] Chain 基本信息:");
        Log($"     numberSegments: {chain.numberSegments}");
        Log($"     numberJoints: {chain.numberJoints}");
        Log($"     totalDoF: {chain.totalDoF}");
        Log($"     rootPosition: {chain.rootPosition}");
        Log($"     rootRotation: {chain.rootRotation.eulerAngles}");
        Log("");
        
        // 3.2 列出所有 segment
        Log("[3.2] 所有 Segments:");
        for (int i = 0; i < chain.segments.Count; i++)
        {
            Segment seg = chain.segments[i];
            ArticulationBody ab = seg.linkedBody;
            Log($"     [{i}] {ab.name}");
            Log($"         jointType: {ab.jointType}");
            Log($"         localPosition: {seg.localPosition}");
            Log($"         jointIndex (axis): {seg.jointIndex}");
        }
        Log("");
        
        // 3.3 eeBody 信息
        Log("[3.3] eeBody (链条终止点):");
        if (chain.eeBody != null)
        {
            Log($"     Name: {chain.eeBody.name}");
            Log($"     JointType: {chain.eeBody.jointType}");
            Log($"     World Position: {chain.eeBody.transform.position}");
            Log($"     Local Position: {chain.eeBody.transform.localPosition}");
            
            // 检查 eeBody 是否在 segments 列表中
            bool eeInSegments = false;
            foreach (Segment seg in chain.segments)
            {
                if (seg.linkedBody == chain.eeBody)
                {
                    eeInSegments = true;
                    break;
                }
            }
            Log($"     eeBody 是否在 segments 列表中: {(eeInSegments ? "是" : "否")}");
            Log("");
            
            // 如果 eeBody 不在 segments 中，说明链在 eeBody 之前停止
            if (!eeInSegments)
            {
                Log("     >>> eeBody 不在 segments 中，链条在到达 eeBody 之前停止");
                Log("     >>> 这意味着 toolVector 需要补偿从最后一个 segment 到 TCP 的距离");
            }
        }
        else
        {
            Log("     !!! eeBody is NULL !!!");
        }
        
        // 3.4 Gripper 信息
        Log("");
        Log("[3.4] GripperTool 信息:");
        GripperTool gripper = chain.Gripper;
        Log($"     toolVector: {gripper.toolVector}");
        Log($"     EndEffectorBody: {(gripper.EndEffectorBody != null ? gripper.EndEffectorBody.name : "NULL")}");
        
        // 3.5 如果有TCP Transform，计算完整链长
        if (tcpTransform != null)
        {
            Log("");
            Log("[3.5] 从基座到TCP的完整路径:");
            
            // 从根到最后一个segment的位置
            Vector3 lastSegmentWorld = chain.segments[chain.segments.Count - 1].linkedBody.transform.position;
            Vector3 rootWorld = chain.rootPosition;
            
            Log($"     Root World: {rootWorld}");
            Log($"     Last Segment World: {lastSegmentWorld}");
            Log($"     TCP World: {tcpTransform.position}");
            
            float rootToLastSeg = Vector3.Distance(rootWorld, lastSegmentWorld);
            float lastSegToTCP = Vector3.Distance(lastSegmentWorld, tcpTransform.position);
            float rootToTCP = Vector3.Distance(rootWorld, tcpTransform.position);
            
            Log("");
            Log($"     Root -> Last Segment: {rootToLastSeg * 1000:F1} mm");
            Log($"     Last Segment -> TCP: {lastSegToTCP * 1000:F1} mm");
            Log($"     Root -> TCP (直线): {rootToTCP * 1000:F1} mm");
        }
    }
    
    #endregion
    
    #region 完整调试
    
    void RunFullDebug()
    {
        logBuilder.Clear();
        
        Log("████████████████████████████████████████████████████████████████");
        Log("█                     TOOL OFFSET DEBUG                         █");
        Log("█                      完整调试报告                              █");
        Log("████████████████████████████████████████████████████████████████");
        Log($"时间: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
        Log("");
        
        // 确保关节状态是最新的
        robot.SyncJointState();
        
        // 运行三个步骤
        RunStep1_ValidateToolVector();
        RunStep2_ValidateFKOutput();
        RunStep3_ValidateChainTermination();
        
        // 总结
        Log("");
        Log("████████████████████████████████████████████████████████████████");
        Log("█                          总结                                 █");
        Log("████████████████████████████████████████████████████████████████");
        Log("");
        
        GripperTool gripper = robot.Articulation.Gripper;
        
        Log("关键数值汇总:");
        Log($"  - toolVector: {gripper.toolVector}");
        Log($"  - toolVector magnitude: {gripper.toolVector.magnitude * 1000:F2} mm");
        Log($"  - Chain segments: {robot.Articulation.numberSegments}");
        Log($"  - Chain joints: {robot.Articulation.numberJoints}");
        Log($"  - eeBody: {(robot.Articulation.eeBody != null ? robot.Articulation.eeBody.name : "NULL")}");
        
        if (tcpTransform != null && eeTransform != null)
        {
            Vector3 actualOffset = eeTransform.InverseTransformPoint(tcpTransform.position);
            Vector3 diff = actualOffset - gripper.toolVector;
            Log($"  - 实际 EE->TCP 偏移: {actualOffset}");
            Log($"  - 与 toolVector 差异: {diff.magnitude * 1000:F2} mm");
            
            if (diff.magnitude > 0.001f)
            {
                Log("");
                Log("!!! 潜在问题检测 !!!");
                Log("toolVector 与实际 EE->TCP 偏移不一致。");
                Log("这可能是 Y 偏移的原因之一。");
                Log("");
                Log("建议修复方法:");
                Log($"  1. 手动设置 toolVector = {actualOffset}");
                Log("  2. 或检查 GripperTool 的自动检测逻辑");
            }
        }
        
        // 打印 toolVector 在 FK 中的应用方式
        Log("");
        Log("FK中toolVector的应用:");
        Log("  在 Chain.JntToCart() 中，toolVector 被添加到最后一个 segment 的 localPosition");
        Log("  计算方式: link_vector = segment.localPosition + toolVector");
        Log("  这假设 toolVector 是在局部坐标系中定义的");
        
        Log("");
        Log("调试完成。日志已自动保存。");
        
        // 自动保存
        SaveLogToFile();
    }
    
    #endregion
    
    #region 单点测试
    
    IEnumerator TestSingleTargetWithOffset()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  单点测试: 验证IK求解后的TCP位置偏移                           ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        Vector3 rootPos = robot.Articulation.rootPosition;
        Vector3 targetWorld = rootPos + testTargetRelative;
        Quaternion targetRot = Quaternion.Euler(testOrientation);
        
        Log($"[输入] 测试目标 (相对基座): {testTargetRelative}");
        Log($"[输入] 测试目标 (世界坐标): {targetWorld}");
        Log($"[输入] 目标朝向 (Euler): {testOrientation}");
        Log($"[输入] 目标Y值: {targetWorld.y:F4}");
        Log("");
        
        // 记录驱动前的关节状态
        robot.SyncJointState();
        Vector<float> beforeJoints = robot.GetCurrentJointState();
        Log($"[驱动前] 关节角 (从物理引擎读取): [{FormatVector(beforeJoints)}]");
        
        // 记录驱动前各关节的实际 jointPosition
        Log("[驱动前] ArticulationBody.jointPosition (弧度):");
        int jIdx = 0;
        foreach (Segment seg in robot.Articulation.segments)
        {
            if (seg.linkedBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                float posRad = seg.linkedBody.jointPosition[0];
                float posDeg = posRad * Mathf.Rad2Deg;
                Log($"     [{jIdx}] {seg.linkedBody.name}: {posRad:F4} rad = {posDeg:F2} deg");
                jIdx++;
            }
        }
        Log("");
        
        // 创建目标位姿
        Matrix4x4 targetPose = Matrix4x4.TRS(targetWorld, targetRot, Vector3.one);
        
        // 求解IK
        robot.SyncJointState();
        Vector<float> initialGuess = robot.GetCurrentJointState();
        
        Log("[IK] 开始求解...");
        Log($"[IK] 初始猜测: [{FormatVector(initialGuess)}]");
        Vector<float> solution = robot.SolveInverseKinematics(targetPose, initialGuess);
        Log($"[IK] 求解完成，关节角: [{FormatVector(solution)}]");
        
        // 获取IK误差
        Vector<float> posError = robot.IKSolver.p_err;
        float posErrMag = Mathf.Sqrt(posError[0] * posError[0] + posError[1] * posError[1] + posError[2] * posError[2]);
        Log($"[IK] 位置误差: {posErrMag * 1000:F3} mm");
        Log($"[IK] 位置误差分量: ({posError[0] * 1000:F3}, {posError[1] * 1000:F3}, {posError[2] * 1000:F3}) mm");
        Log("");
        
        // 获取FK计算的末端位置
        Matrix4x4 fkResult = robot.Articulation.base2EETransform;
        Vector3 fkPosition = new Vector3(fkResult.m03, fkResult.m13, fkResult.m23);
        
        Log("[FK] base2EETransform 位置: " + fkPosition);
        Log($"[FK] base2EETransform Y值: {fkPosition.y:F4}");
        Log("");
        
        // 驱动机器人到求解的关节角
        Log("[运动] 驱动机器人到目标位置...");
        Log($"[运动] 目标关节角: [{FormatVector(solution)}]");
        robot.DriveJoints(solution);
        
        // 等待物理稳定
        yield return new WaitForSeconds(2.0f);
        
        // 获取驱动后的关节状态
        robot.SyncJointState();
        Vector<float> afterJoints = robot.GetCurrentJointState();
        Log($"[驱动后] 关节角 (从物理引擎读取): [{FormatVector(afterJoints)}]");
        
        // 对比目标关节角和实际关节角
        Log("[对比] IK目标 vs 物理实际:");
        for (int i = 0; i < solution.Count && i < afterJoints.Count; i++)
        {
            float diff = solution[i] - afterJoints[i];
            Log($"     关节{i}: 目标={solution[i]:F2}° 实际={afterJoints[i]:F2}° 差={diff:F2}°");
        }
        Log("");
        
        // 记录驱动后各关节的实际 jointPosition
        Log("[驱动后] ArticulationBody.jointPosition (弧度):");
        jIdx = 0;
        foreach (Segment seg in robot.Articulation.segments)
        {
            if (seg.linkedBody.jointType == ArticulationJointType.RevoluteJoint)
            {
                float posRad = seg.linkedBody.jointPosition[0];
                float posDeg = posRad * Mathf.Rad2Deg;
                float targetDeg = solution[jIdx];
                float diff = targetDeg - posDeg;
                Log($"     [{jIdx}] {seg.linkedBody.name}: {posRad:F4} rad = {posDeg:F2} deg (目标: {targetDeg:F2}°, 差: {diff:F2}°)");
                jIdx++;
            }
        }
        Log("");
        
        // 获取稳定后的实际TCP位置
        if (tcpTransform != null)
        {
            Vector3 actualTCPWorld = tcpTransform.position;
            Vector3 actualTCPInBase = Quaternion.Inverse(robot.Articulation.rootRotation) * 
                                       (actualTCPWorld - robot.Articulation.rootPosition);
            
            Log("[稳定后] Unity TCP 世界坐标: " + actualTCPWorld);
            Log("[稳定后] Unity TCP 基座坐标: " + actualTCPInBase);
            Log($"[稳定后] Unity TCP Y值 (世界): {actualTCPWorld.y:F4}");
            Log($"[稳定后] Unity TCP Y值 (基座): {actualTCPInBase.y:F4}");
            Log("");
            
            // 计算偏移
            float targetY = testTargetRelative.y;
            float actualY = actualTCPInBase.y;
            float yOffset = actualY - targetY;
            
            Log("═══════════════════════════════════════");
            Log($">>> 目标 Y = {targetY:F4}");
            Log($">>> 实际 Y = {actualY:F4}");
            Log($">>> Y 偏移 = {yOffset * 1000:F2} mm ({(yOffset < 0 ? "偏下" : "偏上")})");
            Log("═══════════════════════════════════════");
            
            Vector3 totalOffset = actualTCPInBase - testTargetRelative;
            Log($">>> 总位置偏移: ({totalOffset.x * 1000:F2}, {totalOffset.y * 1000:F2}, {totalOffset.z * 1000:F2}) mm");
            Log($">>> 总偏移量: {totalOffset.magnitude * 1000:F2} mm");
            
            // 用实际关节角重新计算FK
            Log("");
            Log("[验证] 用实际关节角重新计算FK:");
            robot.Articulation.UpdateChainKinematics(afterJoints);
            robot.Articulation.JntToCart();
            Matrix4x4 verifyFK = robot.Articulation.base2EETransform;
            Vector3 verifyFKPos = new Vector3(verifyFK.m03, verifyFK.m13, verifyFK.m23);
            Log($"     FK位置 (用实际关节角): {verifyFKPos}");
            Log($"     FK与Unity TCP差值: {(verifyFKPos - actualTCPInBase).magnitude * 1000:F2} mm");
            
            // 逐段对比 FK segment 位置 vs Unity ArticulationBody 位置
            Log("");
            Log("[关键诊断] 逐段对比 FK vs Unity:");
            Chain chain = robot.Articulation;
            for (int i = 0; i < chain.segments.Count; i++)
            {
                Segment seg = chain.segments[i];
                Vector3 fkSegPos = chain.segmentPositions[i];
                Vector3 unitySegPos = seg.linkedBody.transform.position;
                // 转换 Unity 位置到基座坐标系
                Vector3 unitySegInBase = Quaternion.Inverse(chain.rootRotation) * (unitySegPos - chain.rootPosition);
                Vector3 diff = fkSegPos - unitySegInBase;
                
                Log($"     [{i}] {seg.linkedBody.name}:");
                Log($"         FK位置:    ({fkSegPos.x:F4}, {fkSegPos.y:F4}, {fkSegPos.z:F4})");
                Log($"         Unity位置: ({unitySegInBase.x:F4}, {unitySegInBase.y:F4}, {unitySegInBase.z:F4})");
                Log($"         差值:      ({diff.x * 1000:F1}, {diff.y * 1000:F1}, {diff.z * 1000:F1}) mm, 总差: {diff.magnitude * 1000:F1} mm");
                
                // 同时打印关节轴和旋转信息
                if (seg.linkedBody.jointType == ArticulationJointType.RevoluteJoint)
                {
                    Log($"         jointIndex: {seg.jointIndex}");
                    Log($"         localPosition: {seg.localPosition}");
                    Log($"         Unity localPos: {seg.linkedBody.transform.localPosition}");
                }
            }
            
            // 打印 eeBody 和 TCP
            if (chain.eeBody != null)
            {
                Vector3 eeUnityPos = chain.eeBody.transform.position;
                Vector3 eeUnityInBase = Quaternion.Inverse(chain.rootRotation) * (eeUnityPos - chain.rootPosition);
                Log($"     [eeBody] {chain.eeBody.name}:");
                Log($"         Unity位置: ({eeUnityInBase.x:F4}, {eeUnityInBase.y:F4}, {eeUnityInBase.z:F4})");
            }
            Log($"     [TCP] Unity位置: ({actualTCPInBase.x:F4}, {actualTCPInBase.y:F4}, {actualTCPInBase.z:F4})");
            Log($"     [FK终点] base2EE: ({verifyFKPos.x:F4}, {verifyFKPos.y:F4}, {verifyFKPos.z:F4})");
        }
        else
        {
            Log("[稳定后] 无法获取 TCP Transform，请在 Inspector 中设置");
        }
        
        Log("");
        Log("单点测试完成。");
        SaveLogToFile();
    }
    
    #endregion
    
    #region Orientation 对比测试
    
    void RunOrientationComparison()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  Orientation 对比测试: (-90,0,0) vs (180,0,0)                  ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        // 测试不同的 Euler 角设置
        Vector3[] testOrientations = new Vector3[]
        {
            new Vector3(-90f, 0f, 0f),
            new Vector3(180f, 0f, 0f),
            new Vector3(0f, 0f, 0f),
            new Vector3(90f, 0f, 0f),
        };
        
        foreach (Vector3 euler in testOrientations)
        {
            Quaternion rot = Quaternion.Euler(euler);
            
            // 计算各轴在世界坐标系中的方向
            Vector3 xAxis = rot * Vector3.right;   // 工具X轴
            Vector3 yAxis = rot * Vector3.up;      // 工具Y轴
            Vector3 zAxis = rot * Vector3.forward; // 工具Z轴（通常是指向方向）
            
            Log($"Euler ({euler.x:F0}, {euler.y:F0}, {euler.z:F0}):");
            Log($"  X轴 (right)   指向: ({xAxis.x:F2}, {xAxis.y:F2}, {xAxis.z:F2})");
            Log($"  Y轴 (up)      指向: ({yAxis.x:F2}, {yAxis.y:F2}, {yAxis.z:F2})");
            Log($"  Z轴 (forward) 指向: ({zAxis.x:F2}, {zAxis.y:F2}, {zAxis.z:F2})");
            
            // 判断哪个轴指向下方
            string downwardAxis = "";
            if (zAxis.y < -0.9f) downwardAxis = "Z轴指向下方 (工具前端朝下)";
            else if (yAxis.y < -0.9f) downwardAxis = "Y轴指向下方";
            else if (xAxis.y < -0.9f) downwardAxis = "X轴指向下方";
            else if (zAxis.y > 0.9f) downwardAxis = "Z轴指向上方";
            else downwardAxis = "无轴直接指向下方";
            
            Log($"  >>> {downwardAxis}");
            Log("");
        }
        
        Log("结论:");
        Log("- (-90, 0, 0): 使 Z 轴指向 -Y（世界向下）");
        Log("- (180, 0, 0): 使 Y 轴指向 -Y，Z 轴指向 -Z");
        Log("");
        Log("如果你的系统期望工具的 Z 轴作为指向方向，");
        Log("则 (-90, 0, 0) 才能实现 '向下' 姿态。");
        Log("");
        
        // 检查 eeBody 的默认坐标系
        if (robot.Articulation.eeBody != null)
        {
            Transform ee = robot.Articulation.eeBody.transform;
            Log("当前 eeBody 坐标系方向 (世界坐标):");
            Log($"  eeBody.forward (Z): {ee.forward}");
            Log($"  eeBody.up (Y):      {ee.up}");
            Log($"  eeBody.right (X):   {ee.right}");
            
            // 分析当前姿态下，工具应该指向哪里
            Log("");
            Log("如果工具Z轴是工作方向，当前工具指向: " + ee.forward);
            
            // 判断当前是否朝下
            float downDot = Vector3.Dot(ee.forward, Vector3.down);
            if (downDot > 0.9f)
                Log(">>> 当前工具已经朝下");
            else if (downDot < -0.9f)
                Log(">>> 当前工具朝上");
            else
                Log($">>> 当前工具与向下方向夹角: {Mathf.Acos(downDot) * Mathf.Rad2Deg:F1}°");
        }
        
        SaveLogToFile();
    }
    
    #endregion
    
    #region 步骤8: 隔离测试 - 临时禁用 toolVector
    
    IEnumerator TestWithoutToolVector()
    {
        Log("");
        Log("╔══════════════════════════════════════════════════════════════╗");
        Log("║  步骤8: 隔离测试 - 临时禁用 toolVector                          ║");
        Log("╚══════════════════════════════════════════════════════════════╝");
        Log("");
        
        GripperTool gripper = robot.Articulation.Gripper;
        Vector3 originalToolVector = gripper.toolVector;
        
        Log($"原始 toolVector: {originalToolVector}");
        Log("");
        
        // 临时将 toolVector 设为零
        gripper.toolVector = Vector3.zero;
        Log("已将 toolVector 设为 (0,0,0)");
        Log("");
        
        // 执行测试
        Log("执行 IK 求解...");
        Vector3 rootPos = robot.Articulation.rootPosition;
        Vector3 targetWorld = rootPos + testTargetRelative;
        Quaternion targetRot = Quaternion.Euler(testOrientation);
        
        Matrix4x4 targetPose = Matrix4x4.TRS(targetWorld, targetRot, Vector3.one);
        
        robot.SyncJointState();
        Vector<float> initialGuess = robot.GetCurrentJointState();
        Vector<float> solution = robot.SolveInverseKinematics(targetPose, initialGuess);
        
        Matrix4x4 fkResult = robot.Articulation.base2EETransform;
        Vector3 fkPosition = new Vector3(fkResult.m03, fkResult.m13, fkResult.m23);
        
        Log($"目标位置 (基座坐标): {testTargetRelative}");
        Log($"FK结果 (toolVector=0): {fkPosition}");
        
        // 驱动并等待稳定
        robot.DriveJoints(solution);
        yield return new WaitForSeconds(2.0f);
        
        if (tcpTransform != null)
        {
            Vector3 actualTCP = tcpTransform.position;
            Vector3 actualTCPInBase = Quaternion.Inverse(robot.Articulation.rootRotation) * 
                                      (actualTCP - robot.Articulation.rootPosition);
            
            Log($"实际 TCP (toolVector=0): {actualTCPInBase}");
            
            float yOffset = actualTCPInBase.y - testTargetRelative.y;
            Log($"Y 偏移 (toolVector=0): {yOffset * 1000:F2} mm");
        }
        
        // 恢复原始 toolVector
        gripper.toolVector = originalToolVector;
        Log("");
        Log($"已恢复 toolVector: {originalToolVector}");
        Log("");
        
        Log("对比结论:");
        Log("- 如果禁用 toolVector 后偏移消失，问题在 toolVector 的方向/大小");
        Log("- 如果偏移仍在，问题在 FK/IK 核心计算或坐标系定义");
        
        SaveLogToFile();
    }
    
    #endregion
    
    #region 工具方法
    
    void Log(string message)
    {
        string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
        string formatted = $"[{timestamp}] {message}";
        logBuilder.AppendLine(formatted);
        Debug.Log(formatted);
    }
    
    void SaveLogToFile()
    {
        try
        {
            string directory = Path.GetDirectoryName(logFilePath);
            if (!Directory.Exists(directory))
            {
                Directory.CreateDirectory(directory);
            }
            
            File.WriteAllText(logFilePath, logBuilder.ToString());
            Debug.Log($"[ToolOffsetDebugger] Log saved to: {logFilePath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[ToolOffsetDebugger] Failed to save log: {e.Message}");
        }
    }
    
    string FormatVector(Vector<float> vec)
    {
        if (vec == null) return "null";
        
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < vec.Count; i++)
        {
            sb.Append($"{vec[i]:F2}");
            if (i < vec.Count - 1) sb.Append(", ");
        }
        return sb.ToString();
    }
    
    #endregion
    
    #region GUI
    
    void OnGUI()
    {
        if (!isInitialized) return;
        
        GUIStyle boxStyle = new GUIStyle(GUI.skin.box);
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 14;
        labelStyle.normal.textColor = Color.white;
        
        GUILayout.BeginArea(new Rect(10, 400, 300, 220), boxStyle);
        GUILayout.Label("Tool Offset Debugger", GUI.skin.box);
        GUILayout.Label("[D] Run Full Debug (Step 1-3)", labelStyle);
        GUILayout.Label("[T] Test Single Target", labelStyle);
        GUILayout.Label("[O] Orientation Comparison", labelStyle);
        GUILayout.Label("[8] Isolate Test (no toolVector)", labelStyle);
        GUILayout.Label("[L] Save Log", labelStyle);
        GUILayout.Space(10);
        GUILayout.Label($"Target: {testTargetRelative}", labelStyle);
        GUILayout.Label($"Orientation: {testOrientation}", labelStyle);
        GUILayout.EndArea();
    }
    
    #endregion
}
