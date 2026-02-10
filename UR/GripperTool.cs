using UnityEngine;

/// <summary>
/// 夹爪工具类
/// 
/// 管理末端执行器的工具偏移和基本夹爪控制
/// 用于在正运动学计算中添加工具长度补偿
/// </summary>
public class GripperTool
{
    #region 属性
    
    /// <summary>
    /// 关联的末端执行器ArticulationBody
    /// </summary>
    public ArticulationBody EndEffectorBody { get; private set; }
    
    /// <summary>
    /// 工具向量（末端执行器到工具中心点的偏移，局部坐标系）
    /// 通常在Z轴方向（工具指向方向）
    /// </summary>
    public Vector3 toolVector = Vector3.zero;
    
    /// <summary>
    /// 夹爪开启位置（米）
    /// </summary>
    public float GripperOpen { get; set; } = 0.05f;
    
    /// <summary>
    /// 夹爪关闭位置（米）
    /// </summary>
    public float GripperClosed { get; set; } = 0.0f;
    
    /// <summary>
    /// 当前夹爪目标位置
    /// </summary>
    public float GripperGoal { get; set; } = 0.0f;
    
    /// <summary>
    /// 当前夹爪状态 (0=关闭, 1=完全打开)
    /// </summary>
    public float CurrentGripState { get; private set; } = 0.0f;
    
    #endregion
    
    #region 构造函数
    
    /// <summary>
    /// 从ArticulationBody创建GripperTool
    /// </summary>
    /// <param name="eeBody">末端执行器ArticulationBody</param>
    /// <param name="toolOffset">工具偏移向量（可选）</param>
    public GripperTool(ArticulationBody eeBody, Vector3? toolOffset = null)
    {
        EndEffectorBody = eeBody;
        
        if (toolOffset.HasValue)
        {
            toolVector = toolOffset.Value;
        }
        else if (eeBody != null)
        {
            // 尝试自动检测工具偏移
            // 查找名为 "TCP" 或 "tool_center_point" 的子对象
            Transform tcpTransform = FindTCPTransform(eeBody.transform);
            if (tcpTransform != null)
            {
                // 使用世界坐标差值计算，避免 Transform scale 的影响
                // 然后转换到 eeBody 的局部坐标系（不考虑 scale）
                Vector3 worldOffset = tcpTransform.position - eeBody.transform.position;
                
                // 使用旋转的逆来转换到局部坐标系，而不是 InverseTransformPoint（会受 scale 影响）
                toolVector = Quaternion.Inverse(eeBody.transform.rotation) * worldOffset;
                
                Debug.Log($"[GripperTool] Auto-detected tool offset (world): {worldOffset}");
                Debug.Log($"[GripperTool] Auto-detected tool offset (local): {toolVector}");
            }
        }
        
        // 单位检测：如果偏移量大于0.5米，可能有问题
        if (toolVector.magnitude > 0.5f)
        {
            Debug.LogWarning($"[GripperTool] Tool offset seems too large ({toolVector.magnitude:F2}m), please check!");
        }
        
        GripperGoal = GripperOpen;
    }
    
    /// <summary>
    /// 无参构造函数（用于无夹爪的机器人）
    /// </summary>
    public GripperTool()
    {
        EndEffectorBody = null;
        toolVector = Vector3.zero;
    }
    
    #endregion
    
    #region 公共方法
    
    /// <summary>
    /// 设置工具偏移向量
    /// </summary>
    /// <param name="offset">偏移向量（局部坐标系，单位：米）</param>
    public void SetToolOffset(Vector3 offset)
    {
        toolVector = offset;
    }
    
    /// <summary>
    /// 设置工具长度（沿Z轴方向）
    /// </summary>
    /// <param name="length">工具长度（米）</param>
    public void SetToolLength(float length)
    {
        toolVector = new Vector3(0f, 0f, length);
    }
    
    /// <summary>
    /// 打开夹爪
    /// </summary>
    public void Open()
    {
        GripperGoal = GripperOpen;
    }
    
    /// <summary>
    /// 关闭夹爪
    /// </summary>
    public void Close()
    {
        GripperGoal = GripperClosed;
    }
    
    /// <summary>
    /// 设置夹爪目标位置
    /// </summary>
    /// <param name="position">目标位置（0=关闭，GripperOpen=完全打开）</param>
    public void SetGripperPosition(float position)
    {
        GripperGoal = Mathf.Clamp(position, GripperClosed, GripperOpen);
    }
    
    /// <summary>
    /// 获取工具中心点在世界坐标系中的位置
    /// </summary>
    public Vector3 GetTCPWorldPosition()
    {
        if (EndEffectorBody == null) return Vector3.zero;
        return EndEffectorBody.transform.TransformPoint(toolVector);
    }
    
    /// <summary>
    /// 获取工具中心点在世界坐标系中的旋转
    /// </summary>
    public Quaternion GetTCPWorldRotation()
    {
        if (EndEffectorBody == null) return Quaternion.identity;
        return EndEffectorBody.transform.rotation;
    }
    
    #endregion
    
    #region 私有方法
    
    /// <summary>
    /// 查找TCP (Tool Center Point) Transform
    /// </summary>
    private Transform FindTCPTransform(Transform parent)
    {
        if (parent == null) return null;
        
        // 搜索常见的TCP命名
        string[] tcpNames = { "TCP", "tcp", "tool_center_point", "tool0", "ee_link" };
        
        foreach (string name in tcpNames)
        {
            Transform found = parent.Find(name);
            if (found != null) return found;
        }
        
        // 递归搜索子对象
        foreach (Transform child in parent)
        {
            foreach (string name in tcpNames)
            {
                if (child.name.ToLower().Contains(name.ToLower()))
                {
                    return child;
                }
            }
            
            Transform foundInChild = FindTCPTransform(child);
            if (foundInChild != null) return foundInChild;
        }
        
        return null;
    }
    
    #endregion
}
