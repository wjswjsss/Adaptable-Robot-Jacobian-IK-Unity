using UnityEngine;

/// <summary>
/// 可抓取物体脚本
/// 
/// 挂载在可以被夹爪抓取的物体上
/// 管理物体的抓取状态和物理行为
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(Collider))]
public class GraspableObject : MonoBehaviour
{
    #region Inspector 配置
    
    [Header("Grasp Settings")]
    [Tooltip("物体是否可以被抓取")]
    public bool canBeGrasped = true;
    
    [Tooltip("抓取时是否禁用重力（推荐开启以提高稳定性）")]
    public bool disableGravityWhenGrasped = true;
    
    [Header("Visual Feedback")]
    [Tooltip("被抓取时的颜色变化")]
    public Color graspedColor = new Color(0.5f, 1f, 0.5f, 1f);
    
    [Tooltip("原始颜色（自动获取）")]
    private Color originalColor;
    
    #endregion
    
    #region 公共属性
    
    /// <summary>
    /// 当前是否被抓取
    /// </summary>
    public bool IsGrasped { get; private set; } = false;
    
    /// <summary>
    /// 抓取此物体的控制器
    /// </summary>
    public GripperGraspController GraspedBy { get; private set; } = null;
    
    /// <summary>
    /// 物体的 Rigidbody
    /// </summary>
    public Rigidbody Rb { get; private set; }
    
    #endregion
    
    #region 私有变量
    
    private Renderer objectRenderer;
    private bool originalUseGravity;
    private bool originalIsKinematic;
    
    // 初始位置（用于重置）
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    
    #endregion
    
    #region Unity 生命周期
    
    void Awake()
    {
        Rb = GetComponent<Rigidbody>();
        objectRenderer = GetComponent<Renderer>();
        
        if (objectRenderer != null && objectRenderer.material != null)
        {
            originalColor = objectRenderer.material.color;
        }
        
        // 保存初始状态
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        originalUseGravity = Rb.useGravity;
        originalIsKinematic = Rb.isKinematic;
    }
    
    #endregion
    
    #region 公共方法
    
    /// <summary>
    /// 被抓取时调用
    /// </summary>
    /// <param name="controller">抓取此物体的控制器</param>
    public void OnGrasped(GripperGraspController controller)
    {
        if (!canBeGrasped || IsGrasped) return;
        
        IsGrasped = true;
        GraspedBy = controller;
        
        // 物理设置
        if (disableGravityWhenGrasped)
        {
            Rb.useGravity = false;
        }
        
        // 视觉反馈
        if (objectRenderer != null && objectRenderer.material != null)
        {
            objectRenderer.material.color = graspedColor;
        }
        
        Debug.Log($"[GraspableObject] {gameObject.name} grasped by {controller.gameObject.name}");
    }
    
    /// <summary>
    /// 被释放时调用
    /// </summary>
    public void OnReleased()
    {
        if (!IsGrasped) return;
        
        IsGrasped = false;
        GraspedBy = null;
        
        // 恢复物理设置
        Rb.useGravity = originalUseGravity;
        
        // 恢复视觉
        if (objectRenderer != null && objectRenderer.material != null)
        {
            objectRenderer.material.color = originalColor;
        }
        
        Debug.Log($"[GraspableObject] {gameObject.name} released");
    }
    
    /// <summary>
    /// 重置到初始状态
    /// </summary>
    public void ResetToInitial()
    {
        // 如果正在被抓取，先释放
        if (IsGrasped && GraspedBy != null)
        {
            GraspedBy.ForceRelease();
        }
        
        // 重置状态
        IsGrasped = false;
        GraspedBy = null;
        
        // 重置物理
        Rb.velocity = Vector3.zero;
        Rb.angularVelocity = Vector3.zero;
        Rb.useGravity = originalUseGravity;
        Rb.isKinematic = originalIsKinematic;
        
        // 重置位置
        transform.position = initialPosition;
        transform.rotation = initialRotation;
        
        // 重置视觉
        if (objectRenderer != null && objectRenderer.material != null)
        {
            objectRenderer.material.color = originalColor;
        }
    }
    
    /// <summary>
    /// 更新初始位置（用于动态生成的物体）
    /// </summary>
    public void SetInitialPose(Vector3 position, Quaternion rotation)
    {
        initialPosition = position;
        initialRotation = rotation;
        transform.position = position;
        transform.rotation = rotation;
    }
    
    #endregion
    
    #region 静态工厂方法
    
    /// <summary>
    /// 创建一个可抓取的立方体
    /// </summary>
    /// <param name="position">位置</param>
    /// <param name="size">尺寸</param>
    /// <param name="mass">质量</param>
    /// <param name="color">颜色</param>
    /// <returns>创建的 GraspableObject</returns>
    public static GraspableObject CreateCube(Vector3 position, float size, float mass, Color color)
    {
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = "GraspableCube";
        cube.transform.position = position;
        cube.transform.rotation = Quaternion.identity;  // 确保无旋转
        cube.transform.localScale = Vector3.one * size;
        
        // 设置 Rigidbody
        Rigidbody rb = cube.GetComponent<Rigidbody>();
        if (rb == null) rb = cube.AddComponent<Rigidbody>();
        rb.mass = mass;
        rb.drag = 0.5f;
        rb.angularDrag = 0.5f;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        
        // 设置颜色（兼容 URP 和 Built-in）
        Renderer renderer = cube.GetComponent<Renderer>();
        if (renderer != null)
        {
            Material mat = CreateCompatibleMaterial(color);
            if (mat != null)
            {
                renderer.material = mat;
            }
            
            // 配置阴影设置（投射和接收阴影）
            ARMaterialHelper.ConfigureRendererShadows(renderer, castShadows: true, receiveShadows: true);
        }
        
        // 添加 GraspableObject 脚本
        GraspableObject graspable = cube.AddComponent<GraspableObject>();
        graspable.originalColor = color;
        graspable.initialPosition = position;
        graspable.initialRotation = Quaternion.identity;
        
        return graspable;
    }
    
    /// <summary>
    /// 创建一个可抓取的球体
    /// </summary>
    public static GraspableObject CreateSphere(Vector3 position, float diameter, float mass, Color color)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = "GraspableSphere";
        sphere.transform.position = position;
        sphere.transform.rotation = Quaternion.identity;  // 确保无旋转
        sphere.transform.localScale = Vector3.one * diameter;
        
        Rigidbody rb = sphere.GetComponent<Rigidbody>();
        if (rb == null) rb = sphere.AddComponent<Rigidbody>();
        rb.mass = mass;
        rb.drag = 0.5f;
        rb.angularDrag = 0.5f;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        
        Renderer renderer = sphere.GetComponent<Renderer>();
        if (renderer != null)
        {
            Material mat = CreateCompatibleMaterial(color);
            if (mat != null)
            {
                renderer.material = mat;
            }
            
            // 配置阴影设置（投射和接收阴影）
            ARMaterialHelper.ConfigureRendererShadows(renderer, castShadows: true, receiveShadows: true);
        }
        
        GraspableObject graspable = sphere.AddComponent<GraspableObject>();
        graspable.originalColor = color;
        graspable.initialPosition = position;
        graspable.initialRotation = Quaternion.identity;
        
        return graspable;
    }
    
    /// <summary>
    /// 创建兼容 URP 和 Built-in 渲染管线的材质（支持光照）
    /// 
    /// 修复洋红色问题：优先使用 ARMaterialHelper（如果可用）
    /// </summary>
    private static Material CreateCompatibleMaterial(Color color)
    {
        // 方法1：优先使用 ARMaterialHelper（推荐，解决AR环境中的洋红色问题）
        Material mat = ARMaterialHelper.CreateColorMaterial(color);
        if (mat != null)
        {
            Debug.Log($"[GraspableObject] Using ARMaterialHelper, shader: {mat.shader.name}");
            return mat;
        }
        
        // 方法2：从 Primitive 获取默认材质（回退方案）
        Debug.LogWarning("[GraspableObject] ARMaterialHelper failed, falling back to primitive...");
        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Renderer tempRenderer = temp.GetComponent<Renderer>();
        
        if (tempRenderer != null && tempRenderer.sharedMaterial != null)
        {
            // 复制默认材质
            mat = new Material(tempRenderer.sharedMaterial);
            Debug.Log($"[GraspableObject] Using primitive default material, shader: {mat.shader.name}");
            
            // 尝试设置颜色（兼容不同 shader）
            if (mat.HasProperty("_BaseColor"))
            {
                mat.SetColor("_BaseColor", color);
            }
            if (mat.HasProperty("_Color"))
            {
                mat.SetColor("_Color", color);
            }
            mat.color = color;  // 通用设置
            
            // 如果支持光照属性，设置它们
            if (mat.HasProperty("_Smoothness"))
            {
                mat.SetFloat("_Smoothness", 0.5f);
            }
            if (mat.HasProperty("_Metallic"))
            {
                mat.SetFloat("_Metallic", 0f);
            }
            if (mat.HasProperty("_Glossiness"))
            {
                mat.SetFloat("_Glossiness", 0.5f);
            }
        }
        
        Object.DestroyImmediate(temp);
        
        if (mat != null)
        {
            return mat;
        }
        
        Debug.LogError("[GraspableObject] Could not create material! Object may appear magenta.");
        return null;
    }
    
    #endregion
}
