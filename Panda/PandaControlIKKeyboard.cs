using System;
using System.Collections;
using System.Threading;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using System.IO;
using UnityEngine.UI;
using Random = UnityEngine.Random;
using UnityEngine.XR.ARSubsystems;
using UnityEngine.XR.ARFoundation;
using UnityEngine.Networking;
using TMPro;


// Any cs class in the assets folder will be available for Unity scripts, without needing explicit importing

// Control scripts must inherit from MonoBehaviours. Attach your control script (like this one) to the root body of the robot articulation.

public class PandaControlIKKeyboard : MonoBehaviour
{
    // Declare an instance of a panda robot.
    public PandaRobot PD;
    public GameObject camera;
    public GameObject parts;

    public Text IsRecordingText;
    public Text MovementText;

    public Vector3 previousPosition = new Vector3(0.0f, 0.0f, 0.0f);


    public Quaternion finalrotation;


    public float Average_range;
    public Vector3 TempinputHand;
    public bool savedstater = false;

    public Quaternion RotationFinger;
    public Vector3 IndexF;
    public Vector3 ThumbF;


    [SerializeField]
    public List<Vector3> positionsCoord;
    public List<Quaternion> quaternionsToDisplay;
    public ARCameraManager _cameraManager;

    public bool Togglekey = false;

    private bool allowRecord = false;
    public bool isMoving = true;

    public float roll = 0;
    public float pitch = 0;
    public float yaw = 0;


    public bool GripperIsOpen = false;


    // Declare any shared variables. Public variables will be visible in the GUI (under the monobehaviour script dropdown)
    public Vector3 Coordthreshold;

    public Vector3 RangeThreshold;
    public bool ReadyToStart = false;

    // Select which kind of controller to use by setting the appropriate flag:
    public bool ForwardKinematicControl = false;
    public bool InverseKinematicControl = true;
    public bool TorqueControl = false;

    private Button togglerecord;
    private Button SaveKpButton;
    private Button SavedState;


    public Vector3 DefaultVal = new Vector3(0.0f, 0.0f, 0.0f);

    public Vector<float> FKJointPositions;
    private Matrix4x4 IKGoalPose;
    private Queue<Vector<float>> LPFiltVals = new Queue<Vector<float>>(); // low pass filter for acceleration
    private Vector<float> TC_stiffness;
    private Vector<float> TC_damping;

    public Vector3 Coord_hand;
    public Vector2 screenPos;

    public float distance;

    public Vector3 Range_hand;
    int Counter = 0;

    public bool FKControlInitialised = false;
    public bool saveKpPending = false;
    public bool IKControlInitialised = false;
    public bool TorqueControlInitialised = false;

    
    public Lightning light;

    public int frameCount = 0;


    const string k_MaxDistanceName = "_MaxDistance";

    const string k_DisplayRotationPerFrameName = "_DisplayRotationPerFrame";

    readonly int k_MaxDistanceId = Shader.PropertyToID(k_MaxDistanceName);

    readonly int k_DisplayRotationPerFrameId = Shader.PropertyToID(k_DisplayRotationPerFrameName);

    [SerializeField]
    Material depthMaterial;

    [SerializeField]
    float maxEnvDistance  = 5.0f;

    [SerializeField]
    AROcclusionManager m_OcclusionManager;

    Matrix4x4 m_DisplayRotationMatrix = Matrix4x4.identity;

    GameObject ExtrinsicsCanvas;
    GameObject ExtrinsicsCanvasRaw;


    public string StartTime;

    EndEffPos EndEffPosScript;


    public Vector3 curTranslation;


    Quaternion initialRotation;

    void OnEnable()
    {
        _cameraManager.frameReceived += OnCameraFrameEventReceived;
        depthMaterial.SetFloat(k_MaxDistanceId, maxEnvDistance);
        depthMaterial.SetMatrix(k_DisplayRotationPerFrameId, m_DisplayRotationMatrix);
    }

    void OnCameraFrameEventReceived(ARCameraFrameEventArgs cameraFrameEventArgs)
    {
        if (depthMaterial != null)
        {
           m_DisplayRotationMatrix = cameraFrameEventArgs.displayMatrix ?? Matrix4x4.identity;

            depthMaterial.SetMatrix(k_DisplayRotationPerFrameId, m_DisplayRotationMatrix);
        }
    }

    private void SaveImageToGallery(Texture2D source, RenderTexture destination, Material mat)
    {
        if (source != null && destination != null)
        {
            Graphics.Blit(source, destination, mat);

            Texture2D destination2D = toTexture2D(destination);

            byte[] bytes = destination2D.EncodeToPNG();
            string filename = "depth_" + StartTime + "_" + frameCount + ".png";
            string filePath = System.IO.Path.Combine(Application.persistentDataPath, filename);

            NativeGallery.SaveImageToGallery(bytes, "MyGallery", filename);
        }
    }

    Texture2D toTexture2D(RenderTexture rTex)
    {
        Texture2D tex = new Texture2D(rTex.width, rTex.height, TextureFormat.RGB24, false);
        var old_rt = RenderTexture.active;
        RenderTexture.active = rTex;

        tex.ReadPixels(new Rect(0, 0, rTex.width, rTex.height), 0, 0);
        tex.Apply();

        RenderTexture.active = old_rt;
        return tex;
    }

    public void updatescore(Vector3 scoring)
    {
        RangeThreshold = scoring;

    }

    public void updatecoord(Vector3 coord)
    {
        Coordthreshold = coord;
    }

    void Start()
    {

        EndEffPosScript = GameObject.Find("end_effector").GetComponent<EndEffPos>() as EndEffPos;
    
        IsRecordingText = GameObject.Find("IsRecording").GetComponent<Text>();
        MovementText = GameObject.Find("Movement").GetComponent<Text>();

        light = GameObject.FindObjectOfType<Lightning>();
        positionsCoord = new List<Vector3>();
        _cameraManager = FindObjectOfType<ARCameraManager>();
        m_OcclusionManager = FindObjectOfType<AROcclusionManager>();
 
        ExtrinsicsCanvasRaw = GameObject.Find("CamExtrinsicsKP/RawImage");
        ExtrinsicsCanvasRaw.SetActive(false);
    
        StartTime = DateTime.Now.ToString("yyMMdd_HH-mm-ss.ffff");
        
        SaveKpButton = GameObject.Find("SaveKp").GetComponent<Button>();
        SaveKpButton.onClick.AddListener(SaveKpPressed);

        togglerecord = GameObject.Find("StartStop").GetComponent<Button>();
        togglerecord.onClick.AddListener(ToggleRecord);

        PD = new PandaRobot(gameObject); // Initialise our robot (this will use default values for inertias, joint limits, etc)

        // Initialise kinematics and joints
        PD.Articulation.JointUpdate();

        // the PandaRobot class by default uses a basic inverse kinematic solver, described in the FastIterSolve class.
        // We can adjust the number of iterations, maximum convergence error, etc. 
        PD.IKSolver.converge_eps = 1e-4f;

        // Upon import, the initial position of the robot is at a joint singularity, and has some overlap of the collision meshes.
        // We generally move the robot to a neutral position before beginning a control script, to ensure smooth behaviour.

        // PD.q_initial stores the default robot joint state values (as seen in the real hardware)
        // PD.q_goal_state stores a vector of goal joint positions for use by the joint drivers. 

        PD.q_goal_state = Vector<float>.Build.DenseOfVector(PD.q_initial);

        // If you change the values in PD.q_initial or PD.q_goal_state, it is a good idea to make sure they don't violate
        // the maximum/minimum joint angles. The subroutine JointLimitConstraints in the FastIterSolve class will ensure the goal joint positions
        // do not violate the joint constraints, eg:
        PD.IKSolver.JointLimitConstraints(PD.Articulation, PD.q_initial).CopyTo(PD.q_goal_state);
    }


    public void SaveKpPressed()
    {
        savedstater = true;
    }


    public static Vector3 GlobalToLocalPoint(Quaternion rotation, Vector3 translation, Vector3 globalPoint)
    {
        // Step 1: compute the relative position vector
        Vector3 relativePosVec = globalPoint - translation;

        // Step 2: compute the rotation quaternion conjugate
        Quaternion rotationConj = Quaternion.Inverse(rotation) * Quaternion.Euler(0f, 180f, 0f); ;

        // Step 3: transform the relative position vector
        Vector3 transformedVec = rotationConj * relativePosVec;

        // Step 4: add the translation vector to obtain the local point vector3
        Vector3 localPoint = transformedVec + translation;

        return localPoint;
    }


    void Update()
    {


        Coord_hand = light.Getvariable();
        Range_hand = light.Range();
        RotationFinger = light.Rotation();
        IndexF = light.Indexfinger();
        ThumbF = light.ThumbFingers();
        screenPos = light.GetScreenCoords();
        distance = light.GetDistance();
        Average_range = Range_hand.x;

        PD.Articulation.JointUpdate();

        Vector3 currentPosition = PD.Articulation.segments[7].linkedBody.transform.position;

        if ((Coord_hand != DefaultVal) && (currentPosition == previousPosition) && Counter > 0 && (savedstater == true))
        {
            saveKpPending = true;

            IKControlInitialised = false;

            
            if (Average_range < 0.009f)
            {
                PD.UpdateGripperStates(0.0021f);
                GripperIsOpen = false;
                EndEffPosScript.SetGripperOpen(0);

            }
            else
            {
                PD.UpdateGripperStates(0.05f);
                GripperIsOpen = true;
                EndEffPosScript.SetGripperOpen(1);
            }

            initialRotation = PD.Articulation.base2EETransform.rotation;

            curTranslation = GlobalToLocalPoint(PD.Articulation.segments[0].linkedBody.transform.rotation, PD.Articulation.segments[0].linkedBody.transform.position, Coord_hand);
        

            Quaternion rollRotation = Quaternion.AngleAxis(roll, initialRotation * Vector3.right);
            Quaternion pitchRotation = Quaternion.AngleAxis(pitch, initialRotation * Vector3.up);
            Quaternion yawRotation = Quaternion.AngleAxis(yaw, initialRotation * Vector3.forward);

            finalrotation = initialRotation * yawRotation * pitchRotation * rollRotation;


            IKGoalPose = Matrix4x4.TRS(curTranslation, finalrotation, new Vector3(1, 1, 1));

            savedstater = false;

            Counter += 1;
        }


        PD.Articulation.JointUpdate();

        if (Counter == 0)
        {

            Vector3 positionSensor = new Vector3(PD.Articulation.base2EETransform.m03, PD.Articulation.base2EETransform.m13, PD.Articulation.base2EETransform.m23);
            IKGoalPose = Matrix4x4.TRS(positionSensor, PD.Articulation.base2EETransform.rotation, new Vector3(1, 1, 1));

            Counter = Counter + 1;

        }
        else
        {
    
            previousPosition = currentPosition;

        }


        // Make sure the robot is in a neutral position before we begin to control it
        if (!ReadyToStart)
        {

            PD.DriveJointsIncremental(PD.Articulation.jointState, PD.q_goal_state);

            // When we have reached the neutral starting position, set the flag and the robot will now transition to using
            // the active controller.

            if ((PD.q_goal_state - PD.Articulation.jointState).L2Norm() < 0.5f)
            {
                isMoving = false;
                ReadyToStart = true;
            }
        }
        else if (InverseKinematicControl)
        {
            /* Set a goal end effector position and rotation, use the built-in iterative inverse kinematic solver to find an appropriate
            * set of joint positions, given the current joint state.
            * If the solver cannot find an appropriate set of joint angles within the given maximum iterations, it will send up an warning in
            * the GUI console and use the last entry in the iteration as the goal joint positions
            */

            if (!IKControlInitialised)
            {
                Vector<float> q_update = PD.SolveInverseKinematics(IKGoalPose, PD.Articulation.jointState);
                q_update.CopyTo(PD.q_goal_state);
                IKControlInitialised = true;

            }


            PD.DriveJointsIncremental(PD.Articulation.jointState, PD.q_goal_state);



            if ((PD.q_goal_state - PD.Articulation.jointState).L2Norm() < 0.5f)
            {
                isMoving = false;
                MovementText.text = "Reached goal state " + (PD.q_goal_state - PD.Articulation.jointState).L2Norm();
            } else {
                isMoving = true;
                MovementText.text = "Reaching Goal State " + (PD.q_goal_state - PD.Articulation.jointState).L2Norm();
            }


        }
        

        if (allowRecord && frameCount % 10 == 0 && (isMoving || saveKpPending)) {
            SaveAll();
        }

        IsRecordingText.text = "RECORDING ALLOWED: " + allowRecord;
        frameCount++;

    }

    public void ToggleRecord() {
       allowRecord = !allowRecord;
    }

    public void SaveAll() {
        StartCoroutine(SeqStart());
    }

    IEnumerator SeqStart() {

        if (saveKpPending && !isMoving) {
            EndEffPosScript.SetKeypointFrame(1);
            saveKpPending = false;
        }
        ExtrinsicsCanvasRaw.SetActive(true);
        yield return StartCoroutine(SaveKPPhoto());
        yield return StartCoroutine(CaptureDepth());
        yield return StartCoroutine(TakeAPhoto());
        EndEffPosScript.SetKeypointFrame(0);
    }


     public void CaptureDepthImage()
    {
        StartCoroutine(CaptureDepth());
    }

    IEnumerator CaptureDepth() {
        yield return new WaitForEndOfFrame();

        Camera camera = Camera.main;
        int width = Screen.width;
        int height = Screen.height;

        Texture2D envDepth = m_OcclusionManager.environmentDepthTexture;

        RenderTexture rt = new RenderTexture(width, height, 24);
        SaveImageToGallery(envDepth, rt, depthMaterial);

        Destroy(rt);
    }



    public void SaveKPText()
    {
        StartCoroutine(SaveKPPhoto());
    }


    IEnumerator SaveKPPhoto() {

        yield return new WaitForEndOfFrame();
        

        Camera camera = Camera.main;
        int width = Screen.width;
        int height = 275;

        RenderTexture rt = new RenderTexture(Screen.width, Screen.height, 24);
        camera.targetTexture = rt;

        var currentRT = RenderTexture.active;
        RenderTexture.active = rt;

        camera.Render();

        Texture2D image = new Texture2D(width, height);
        image.ReadPixels(new Rect(0, Screen.height - height, width, height), 0, 0);
        image.Apply();

        camera.targetTexture = null;

        RenderTexture.active = currentRT;

        byte[] bytes = image.EncodeToPNG();
        string fileName = "text_" + StartTime + "_" + frameCount + ".png";
        NativeGallery.SaveImageToGallery(bytes, "MyGallery", fileName);

        Destroy(rt);
        Destroy(image);
        ExtrinsicsCanvasRaw.SetActive(false);
    }

    public void TakePhoto() {
        StartCoroutine(TakeAPhoto());
    }

    IEnumerator TakeAPhoto() {

        yield return new WaitForEndOfFrame();

        Camera camera = Camera.main;
        int width = (int) (Screen.width / 2.8);
        int height = (int) (Screen.height / 2.8);

        RenderTexture rt = new RenderTexture(width, height, 24);
        camera.targetTexture = rt;

        var currentRT = RenderTexture.active;
        RenderTexture.active = rt;

        camera.Render();

        Texture2D image = new Texture2D(width, height);
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        image.Apply();

        camera.targetTexture = null;

        RenderTexture.active = currentRT;

        byte[] bytes = image.EncodeToPNG();
        string fileName = StartTime + "_" + frameCount + ".png";

        NativeGallery.SaveImageToGallery(bytes, "MyGallery", fileName);
        
        Destroy(rt);
        Destroy(image);
    }


}