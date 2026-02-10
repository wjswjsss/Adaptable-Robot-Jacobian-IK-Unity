using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// 夹爪抓取控制器
/// 
/// 挂载在 HandE 或夹爪区域
/// 负责：
/// - 检测可抓取物体（使用距离检测，因为 ArticulationBody 上 Trigger 不可靠）
/// - 根据 PincherController 的 grip 值判断抓取状态
/// - 使用父子关系 + Kinematic 实现稳定抓取
/// </summary>
public class GripperGraspController : MonoBehaviour
{
    #region Inspector 配置
    
    [Header("References")]
    [Tooltip("夹爪控制器（PincherController）")]
    public PincherController pincherController;
    
    [Tooltip("抓取连接点（物体会附着到这个点，通常是 TCP 或 HandE）")]
    public Transform graspAttachPoint;
    
    [Header("Grasp Settings")]
    [Tooltip("触发抓取的 grip 阈值（0-1）")]
    [Range(0.3f, 0.9f)]
    public float graspThreshold = 0.5f;
    
    [Tooltip("触发释放的 grip 阈值（0-1）")]
    [Range(0.1f, 0.5f)]
    public float releaseThreshold = 0.2f;
    
    [Tooltip("抓取检测区域半径")]
    public float graspZoneRadius = 0.05f;
    
    [Tooltip("检测物体的 Layer（-1 表示所有层）")]
    public LayerMask detectionLayer = -1;
    
    [Header("Debug")]
    [Tooltip("显示抓取区域 Gizmo")]
    public bool showGraspZone = true;
    
    [Tooltip("详细日志")]
    public bool verboseLog = true;
    
    #endregion
    
    #region 公共属性
    
    /// <summary>
    /// 当前是否正在抓取
    /// </summary>
    public bool IsGrasping { get; private set; } = false;
    
    /// <summary>
    /// 当前抓取的物体
    /// </summary>
    public GraspableObject GraspedObject { get; private set; } = null;
    
    /// <summary>
    /// 抓取区域内的物体列表
    /// </summary>
    public List<GraspableObject> ObjectsInZone { get; private set; } = new List<GraspableObject>();
    
    #endregion
    
    #region 私有变量
    
    // 抓取状态保存
    private Transform originalParent;
    private bool wasKinematic;
    private bool wasUsingGravity;
    
    // 距离检测缓冲区
    private Collider[] detectionBuffer = new Collider[20];
    
    #endregion
    
    #region Unity 生命周期
    
    void Start()
    {
        // 验证配置
        if (pincherController == null)
        {
            pincherController = GetComponent<PincherController>();
            if (pincherController == null)
            {
                pincherController = GetComponentInParent<PincherController>();
            }
        }
        
        if (pincherController == null)
        {
            Debug.LogError("[GripperGraspController] PincherController not found!");
        }
        
        // 设置连接点
        if (graspAttachPoint == null)
        {
            // 优先使用 TCP
            Transform tcp = transform.Find("TCP");
            if (tcp != null)
            {
                graspAttachPoint = tcp;
            }
            else
            {
                graspAttachPoint = transform;
            }
        }
        
        if (verboseLog)
        {
            Debug.Log($"[GripperGraspController] Initialized on {gameObject.name}");
            Debug.Log($"[GripperGraspController] Attach point: {graspAttachPoint.name}");
        }
    }
    
    void FixedUpdate()
    {
        if (pincherController == null) return;
        
        // 使用距离检测更新区域内物体
        UpdateObjectsInZoneByDistance();
        
        float currentGrip = pincherController.grip;
        
        // 检查抓取条件
        if (!IsGrasping && currentGrip >= graspThreshold)
        {
            TryGrasp();
        }
        // 检查释放条件
        else if (IsGrasping && currentGrip <= releaseThreshold)
        {
            Release();
        }
        
        // 如果正在抓取，确保物体跟随（物体已经是子对象，会自动跟随）
    }
    
    #endregion
    
    #region 距离检测
    
    /// <summary>
    /// 使用距离检测更新区域内的物体
    /// </summary>
    void UpdateObjectsInZoneByDistance()
    {
        Vector3 center = graspAttachPoint != null ? graspAttachPoint.position : transform.position;
        
        // 使用 OverlapSphere 检测
        int count = Physics.OverlapSphereNonAlloc(center, graspZoneRadius, detectionBuffer, detectionLayer);
        
        // 清空旧列表
        ObjectsInZone.Clear();
        
        for (int i = 0; i < count; i++)
        {
            Collider col = detectionBuffer[i];
            if (col == null) continue;
            
            GraspableObject graspable = col.GetComponent<GraspableObject>();
            if (graspable != null && graspable.canBeGrasped && !graspable.IsGrasped)
            {
                if (!ObjectsInZone.Contains(graspable))
                {
                    ObjectsInZone.Add(graspable);
                }
            }
        }
    }
    
    #endregion
    
    #region 抓取控制
    
    /// <summary>
    /// 尝试抓取区域内的物体
    /// </summary>
    void TryGrasp()
    {
        if (ObjectsInZone.Count == 0)
        {
            if (verboseLog)
            {
                Debug.Log("[GripperGraspController] No objects in grasp zone to grasp");
            }
            return;
        }
        
        // 选择最近的物体
        GraspableObject closest = GetClosestObject();
        if (closest == null || closest.IsGrasped)
        {
            if (verboseLog)
            {
                Debug.Log("[GripperGraspController] No valid object to grasp");
            }
            return;
        }
        
        // 执行抓取
        GraspedObject = closest;
        IsGrasping = true;
        
        // 保存原始状态
        originalParent = GraspedObject.transform.parent;
        
        Rigidbody rb = GraspedObject.Rb;
        wasKinematic = rb.isKinematic;
        wasUsingGravity = rb.useGravity;
        
        // 停止物体运动
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        // 设置物体为 Kinematic（防止物理干扰）
        rb.isKinematic = true;
        rb.useGravity = false;
        
        // 设为抓取点的子对象（保持世界位置）
        GraspedObject.transform.SetParent(graspAttachPoint, true);
        
        // 通知物体
        GraspedObject.OnGrasped(this);
        
        if (verboseLog)
        {
            Debug.Log($"[GripperGraspController] ✓ Grasped: {GraspedObject.gameObject.name}");
            Debug.Log($"[GripperGraspController]   Attached to: {graspAttachPoint.name}");
        }
    }
    
    /// <summary>
    /// 释放当前抓取的物体
    /// </summary>
    public void Release()
    {
        if (!IsGrasping || GraspedObject == null) return;
        
        if (verboseLog)
        {
            Debug.Log($"[GripperGraspController] Releasing: {GraspedObject.gameObject.name}");
        }
        
        // 解除父子关系
        GraspedObject.transform.SetParent(originalParent, true);
        
        // 恢复物理状态
        Rigidbody rb = GraspedObject.Rb;
        rb.isKinematic = wasKinematic;
        rb.useGravity = wasUsingGravity;
        
        // 清零速度
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        // 通知物体
        GraspedObject.OnReleased();
        
        GraspedObject = null;
        IsGrasping = false;
    }
    
    /// <summary>
    /// 强制释放（用于重置等情况）
    /// </summary>
    public void ForceRelease()
    {
        if (GraspedObject != null)
        {
            // 解除父子关系
            GraspedObject.transform.SetParent(null, true);
            
            // 恢复物理
            Rigidbody rb = GraspedObject.Rb;
            if (rb != null)
        {
                rb.isKinematic = false;
                rb.useGravity = true;
            }
            
            GraspedObject = null;
        }
        
        IsGrasping = false;
    }
    
    #endregion
    
    #region 辅助方法
    
    /// <summary>
    /// 获取区域内最近的可抓取物体
    /// </summary>
    GraspableObject GetClosestObject()
    {
        if (ObjectsInZone.Count == 0) return null;
        
        // 清理无效引用
        ObjectsInZone.RemoveAll(obj => obj == null);
        
        if (ObjectsInZone.Count == 0) return null;
        if (ObjectsInZone.Count == 1) return ObjectsInZone[0];
        
        GraspableObject closest = null;
        float minDist = float.MaxValue;
        Vector3 graspCenter = graspAttachPoint != null ? graspAttachPoint.position : transform.position;
        
        foreach (var obj in ObjectsInZone)
        {
            if (obj == null || obj.IsGrasped) continue;
            
            float dist = Vector3.Distance(obj.transform.position, graspCenter);
            if (dist < minDist)
            {
                minDist = dist;
                closest = obj;
            }
        }
        
        return closest;
    }
    
    /// <summary>
    /// 手动检查并尝试抓取（可从外部调用）
    /// </summary>
    public void ManualGrasp()
    {
        if (!IsGrasping)
        {
            UpdateObjectsInZoneByDistance();
            TryGrasp();
        }
    }
    
    #endregion
    
    #region Gizmos
    
    void OnDrawGizmosSelected()
    {
        if (!showGraspZone) return;
        
        Transform centerTransform = graspAttachPoint != null ? graspAttachPoint : transform;
        Vector3 center = centerTransform.position;
        
        // 绘制抓取区域
        Gizmos.color = IsGrasping ? new Color(0f, 1f, 0f, 0.3f) : new Color(1f, 1f, 0f, 0.3f);
        Gizmos.DrawSphere(center, graspZoneRadius);
        
        Gizmos.color = IsGrasping ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(center, graspZoneRadius);
        
        // 绘制检测到的物体
        if (Application.isPlaying)
        {
            foreach (var obj in ObjectsInZone)
            {
                if (obj != null)
        {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawLine(center, obj.transform.position);
                    Gizmos.DrawWireSphere(obj.transform.position, 0.01f);
                }
            }
        }
        
        // 如果正在抓取，绘制连接线
        if (IsGrasping && GraspedObject != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(center, GraspedObject.transform.position);
        }
    }
    
    #endregion
}
