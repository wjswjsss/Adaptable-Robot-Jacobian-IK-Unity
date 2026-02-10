using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// Chain

// To do: add destruction, clear, etc

public class Chain
{
	/* Chain is built from (but does not inherit from) a set of linked ArticulationBodies
    *
    * Given a root gameObject, chain iterates through its children to find any articulation bodies and add them to the chain.
	* It stops when either there are no more children, or a stop condition is triggered by naming one of the children "end_effector"
    *
	* Note on frames and transforms:
	* Can access current local and world position/rotation from gameObject.
    * Chain constructor builds from an existing ArticulationBody, but copies data to independent variables (not pointers) so that ProjectChain
    * is not based on current chain state. To repopulate chain variables with current AB state, call JointUpdate.
	*/

	// TODO: implement a frame class to streamline transform representations
	// TODO: modify jointCreate so different joints can have different limits on velocities.
  
        // publically available variables:
	public int numberSegments = 0;
	public int numberJoints = 0;
	public int totalDoF;
	public List<Segment> segments = new List<Segment>();
	public List<int> dof_indices = new List<int>();
	public Dictionary<int, Segment> chainMap = new Dictionary<int, Segment>();
	private float dist_from_base = 0;

	// TODO: at some point, put all these things together in a Frame class which lets us quickly convert between different representations.
	public Vector3[] segmentPositions;
	public Quaternion[] segmentRotations;
	public Matrix4x4 base2EETransform;
	public ArticulationBody eeBody;
	
	public Vector<float> jointState;

	public Vector2[] jointLimits;
	public float[] jointVelLimit; // arbitrary right now.
	public float jac_condition_eps = 1e-6f;

	// this is a bit redundant but lets us easily access root data if we have multiple robots in a scene
	public Vector3 rootPosition;
	public Quaternion rootRotation;
	public GripperTool Gripper;


	/// <summary>
	/// 末端执行器停止名称列表（链遍历到这些名称时停止）
	/// </summary>
	private static readonly string[] DefaultEEStopNames = { "end_effector", "TCP", "tcp", "HandE", "hand", "ee_link", "tool0" };
	
	public Chain(GameObject rootObject, Vector<float> jointMin, Vector<float> jointMax, float jointVelMax = 20.0f, string[] customEENames = null)
	{
		// 使用自定义名称或默认名称
		string[] eeStopNames = customEENames ?? DefaultEEStopNames;
		
		Component[] childrenBodies = rootObject.GetComponentsInChildren<ArticulationBody>(); // this is recursive, childrenBodies includes itself!
		
		Debug.Log($"[Chain] Found {childrenBodies.Length} ArticulationBodies in hierarchy");

		int chain_i = 0;

		foreach (ArticulationBody cBody in childrenBodies)
		{
			// 检查是否是末端执行器（停止条件）
			bool isEndEffector = false;
			foreach (string eeName in eeStopNames)
			{
				if (cBody.name.Contains(eeName))
				{
					isEndEffector = true;
					break;
				}
			}
			
			if (isEndEffector)
            {
				eeBody = cBody;
				Debug.Log($"[Chain] Found end effector: {cBody.name}, stopping chain traversal");
				Debug.Log($"[Chain] Total segments added: {numberSegments}");
                break;
            }
            
            // 添加到链中
            Debug.Log($"[Chain] Adding segment [{chain_i}]: {cBody.name}, JointType: {cBody.jointType}");
			this.AddSegment(cBody, chain_i);
			if (chain_i < 1)
			{
				ArticulationBody rootBody= cBody;
				totalDoF = rootBody.GetDofStartIndices(dof_indices);

				this.rootPosition = rootObject.transform.position;
				this.rootRotation = rootObject.transform.rotation;
			}
			chain_i++;
			this.numberSegments++;
		}
		
		Debug.Log($"[Chain] Final segment count: {numberSegments}");

		JointCreate(jointMin, jointMax, jointVelMax); // add joint limits during joint state creation
		
		Debug.Log($"[Chain] Final joint count: {numberJoints}");

		// 计算从最后一个 segment 到 TCP 的完整偏移
		// 这包括：LastSegment -> eeBody -> TCP 的路径
		Gripper = CreateGripperWithCorrectOffset();

		// get maximum reachable envelope (due to joint orientations and limits, actual reachable workspace will always be less than this radius)

		foreach (Segment seg in segments)
		{
			dist_from_base += seg.linkedBody.transform.localPosition.magnitude;
		}

	}

	/// <summary>
	/// 创建 GripperTool 并计算正确的 toolVector
	/// toolVector 应该补偿从最后一个 segment 到 TCP 的完整路径
	/// </summary>
	private GripperTool CreateGripperWithCorrectOffset()
	{
		GripperTool gripper = new GripperTool(this.eeBody);
		
		if (this.segments.Count == 0 || this.eeBody == null)
		{
			Debug.LogWarning("[Chain] Cannot calculate correct tool offset: no segments or eeBody");
			return gripper;
		}
		
		// 获取最后一个 segment
		Segment lastSegment = this.segments[this.segments.Count - 1];
		Transform lastSegmentTransform = lastSegment.linkedBody.transform;
		
		// 查找 TCP Transform
		Transform tcpTransform = FindTCPTransform(this.eeBody.transform);
		
		if (tcpTransform == null)
		{
			Debug.Log("[Chain] No TCP found, using eeBody position as TCP");
			// 如果没有找到 TCP，计算从最后一个 segment 到 eeBody 的偏移
			Vector3 worldOffset = this.eeBody.transform.position - lastSegmentTransform.position;
			gripper.toolVector = Quaternion.Inverse(lastSegmentTransform.rotation) * worldOffset;
		}
		else
		{
			// 计算从最后一个 segment 到 TCP 的世界坐标偏移
			Vector3 worldOffset = tcpTransform.position - lastSegmentTransform.position;
			
			// 转换到最后一个 segment 的局部坐标系
			// 注意：FK 中 toolVector 是在最后一个 segment 的局部坐标系中应用的
			gripper.toolVector = Quaternion.Inverse(lastSegmentTransform.rotation) * worldOffset;
			
			Debug.Log($"[Chain] Last segment: {lastSegment.linkedBody.name}");
			Debug.Log($"[Chain] Last segment world pos: {lastSegmentTransform.position}");
			Debug.Log($"[Chain] TCP world pos: {tcpTransform.position}");
			Debug.Log($"[Chain] World offset: {worldOffset}");
			Debug.Log($"[Chain] Local offset (toolVector): {gripper.toolVector}");
		}
		
		return gripper;
	}
	
	/// <summary>
	/// 查找 TCP Transform（递归搜索）
	/// </summary>
	private Transform FindTCPTransform(Transform parent)
	{
		if (parent == null) return null;
		
		string[] tcpNames = { "TCP", "tcp", "tool_center_point", "tool0", "ee_link" };
		
		// 先在直接子对象中搜索
		foreach (string name in tcpNames)
		{
			Transform found = parent.Find(name);
			if (found != null) return found;
		}
		
		// 递归搜索所有子对象
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


	public void JointCreate(Vector<float> jointMin, Vector<float> jointMax, float jointVelMax = 20.0f)
	{
		Vector<float> jointTemp = Vector<float>.Build.Dense(this.numberSegments);
		int i = 0; // joint count
		List<Vector2> BuildJointLimits = new List<Vector2>();
		List<float> BuildVelLimits = new List<float>();
		
		int expectedJoints = jointMin.Count;

		foreach (Segment seg in this.segments)
		{
			if (seg.joint.jointType != ArticulationJointType.FixedJoint)
			{
				// 检查是否超出预期关节数
				if (i >= expectedJoints)
				{
					Debug.LogWarning($"[Chain] Found more movable joints ({i + 1}) than expected ({expectedJoints}). Stopping at joint limit.");
					Debug.LogWarning($"[Chain] Extra joint found in segment: {seg.linkedBody.name}");
					break;
				}
				
				jointTemp[i] = seg.linkedBody.jointPosition[0];

				// may not ever need to access limits at the segment level but just in case
				seg.joint.jointLimits[0] = jointMin[i];
				seg.joint.jointLimits[1] = jointMax[i];

				ArticulationDrive jointDrive = seg.linkedBody.xDrive;
                jointDrive.lowerLimit = jointMin[i];
				jointDrive.upperLimit = jointMax[i];
				seg.linkedBody.xDrive = jointDrive;


				// Limits included as a chain element are useful for building IK elements:
				BuildJointLimits.Add(new Vector2(jointMin[i], jointMax[i]));
				BuildVelLimits.Add(jointVelMax);
				seg.jointCount = i;
				chainMap.Add(i, seg);
				i++;
			}
		}

		this.jointLimits = BuildJointLimits.ToArray();
		this.jointVelLimit = BuildVelLimits.ToArray();
		this.numberJoints = i;

		// copy only a subset of temporary joint vector, 0 to i
		jointState = Vector<float>.Build.Dense(i);
		for (i = 0; i < jointState.Count; i++) { jointState[i] = jointTemp[i]; } // there has GOT to be a better way to do this
	}

	public void JointUpdate()
	{
		// update jointState from articulation body.
        // TODO: implement prismatic joint handler (!)
		int i = 0;
		foreach (Segment seg in this.segments)
		{
			if (seg.linkedBody.jointType != ArticulationJointType.FixedJoint)
			{
				jointState[i] = Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
				i++;
			}
		}
		UpdateChainKinematics(jointState); // ok this is actually doing something, which is more of a problem than if it wasn't.
		JntToCart();
	}

	public Segment getSegment(int seg)
	{
		return this.segments[seg];
	}

	public void AddSegment(ArticulationBody newBody, int i)
	{

		Segment newSegment = new Segment(newBody, i);

		if (i > 0) { segments[i - 1].AddChild(newSegment); }

		this.segments.Add(newSegment);
	}

	public int GetNbrSegments()
	{
		return this.numberSegments;
	}

	public int GetNbrJoints()
	{
		return this.numberJoints;
	}

	public int InsideMaxWorkRadius(Vector3 InputPosition)
	{
		// checkes whether a given world coordinate is reachable given the robot kinematic lengths only
		// returns 1 if theoretically reachable, 0 if not

		// note that this method is a fast jointspace check and does not confirm IK solvability, nor does it take joint limits into account
		// more precise workspace checks can be handled in robot-specific classes

		float point_in_base = (InputPosition - rootPosition).magnitude;
		if (point_in_base < dist_from_base) { return 1; }
		else { return 0; }

	}


	public void UpdateChainState()
    {
        // goes through segments and resets local positions, rotations, ensures
        // projections are reset to current state
        foreach (Segment seg in segments)
        {
			seg.localPosition = seg.linkedBody.transform.localPosition;
			// seg.localRotation = seg.linkedBody.transform.localRotation; // this should be rotation angle -> jointIndex
			seg.segmentPosition = seg.linkedBody.transform.localPosition; // assume we start at 0 on initialisation
			seg.segmentRotation = seg.linkedBody.transform.localRotation;
		}
		JntToCart();
	}


	public void UpdateChainKinematics(Vector<float> q_seed)
	{
        // Use this method to either update the kinematic projection, or ensure the kinematic projection is in line with current
        // joint positions (if called with current joint state).

		// This method updates the projected chain joint positions but does not reference
		// the existing AB, so is independent of jointUpdate (when we need it to be).
        // Use it to iteratively project joint states.

		// TODO: fix spherical joints

		int i = 0;
		foreach (Segment seg in segments)
		{

			ArticulationJointType switchType = seg.linkedBody.jointType;

			switch (switchType)
			{
				case ArticulationJointType.PrismaticJoint:
					{
						// update along joint axis
						Vector3 posUpdate = seg.localPosition + (q_seed[i] - seg.localJointState) * seg.jointIndex;
						seg.localPosition = posUpdate;
						seg.localJointState = q_seed[i];
						i++;
						break;
					}

				case ArticulationJointType.RevoluteJoint:
					{
						// Generate rotation update
						Quaternion rotUpdate = Quaternion.AngleAxis(q_seed[i], seg.jointIndex);

						rotUpdate = seg.segmentRotation * rotUpdate;
						seg.localRotation = rotUpdate;
						seg.localJointState = q_seed[i];

						i++;
						break;
					}

				case ArticulationJointType.SphericalJoint:
					{
						// need to use actual spherical axis-angle representation here
						Quaternion r0 = Quaternion.AngleAxis(seg.localJointState, seg.jointIndex);

						Quaternion rotUpdate = Quaternion.AngleAxis(q_seed[i], seg.jointIndex) * (Quaternion.Inverse(r0) * seg.localRotation);
						seg.localRotation = rotUpdate;
						seg.localJointState = q_seed[i];
						i++;
						break;
					}

				default:
					{

						// do nothing
						break;
					}

			}
		}
	}

	public void JntToCart()
	{
		// Forward projects joint positions to cartesian positions based on localPosition, localRotation
		// If calling for a joint projection, call UpdateJointKinematics first to incorporate new joint angles

		int segmentLength = this.numberSegments;
		var segmentT = new Vector3[segmentLength];
		var segmentRot = new Quaternion[segmentLength];

		// Do some error catching here at some point

		segmentT[0] = this.rootPosition;
		segmentRot[0] = this.rootRotation;
		Vector3 scale = new Vector3(1, 1, 1);

		/* Transform has to be linked to a gameObject, so we can't use this class to write projections. (unless we made phantom gameObjects)
		 * Can use Matrix4x4 (this is a sealed type, so can't inherit).
		 */

		Matrix4x4 rootTransform = Matrix4x4.TRS(segmentT[0], segmentRot[0], scale);
		Matrix4x4 iterTransform = rootTransform;

		// Must initialise to root position to get initial robot orientation correct, BUT this does not seem to pass over the root world position for some reason?
        // iterTransform should be the end of the linked body in the base robot frame.

		// 先计算所有关节的 FK（不包含 toolVector）
        for (int i = 1; i < segmentLength; i++)
		{
			iterTransform = FwKinSegment(iterTransform, this.segments[i]);
			segmentRot[i] = iterTransform.rotation; 
			segmentT[i] = new Vector3(iterTransform.m03, iterTransform.m13, iterTransform.m23);
		}

		this.segmentPositions = segmentT;
		this.segmentRotations = segmentRot;

		// 在 FK 计算完成后，将 toolVector 作为末端局部偏移应用
		// toolVector 是在最后一个 segment 的局部坐标系中定义的
		// 需要先用末端旋转将其转换到世界坐标系，再加到位置上
		if (this.Gripper != null && this.Gripper.toolVector != Vector3.zero)
		{
			// 获取最后一个 segment 的旋转（世界坐标系）
			Quaternion endRotation = iterTransform.rotation;
			
			// 将 toolVector 从局部坐标系转换到世界坐标系
			Vector3 toolOffsetWorld = endRotation * this.Gripper.toolVector;
			
			// 创建 tool offset 的变换矩阵（只有平移）
			Matrix4x4 toolTransform = Matrix4x4.Translate(toolOffsetWorld);
			
			// 应用到末端变换
			iterTransform = toolTransform * iterTransform;
		}

		this.base2EETransform = iterTransform;
	}

	public Matrix4x4 FwKinSegment(Matrix4x4 runningTransform, Segment segment, Vector3 pVector = default(Vector3), float delta_q_i=0.0f)
	{
		// check joint type, calculate appropriate transform updates
		// TODO: need to make an overload method to handle spherical joints
		// TODO: implement a frame class and update this method
		// TODO: are there faster methods for doing transform convolutions?

		// TODO Debug: recheck link vectors and local transforms at initialisation.
		// given we generally don't use delta_q_i, should we take this out?

		Matrix4x4 segmentTransform = Matrix4x4.identity;

		ArticulationJointType switchType = segment.linkedBody.jointType;

		switch (switchType)
		{
			case ArticulationJointType.PrismaticJoint:
				{

					Vector3 joint_displ = delta_q_i *  segment.jointIndex;
					Vector3 link_vector = segment.localPosition + pVector;

					Matrix4x4 localTransform = Matrix4x4.TRS(joint_displ + link_vector , segment.localRotation, new Vector3(1, 1, 1));
					segmentTransform = runningTransform * localTransform;
					break;
				}

			case ArticulationJointType.RevoluteJoint:
				{

					Quaternion joint_displ = Quaternion.AngleAxis(delta_q_i, segment.jointIndex);
					Vector3 link_vector = segment.localPosition + pVector; // Cartesian position relative to joint anchor 
					Matrix4x4 localRotationMatrix = Matrix4x4.Rotate(segment.localRotation) * Matrix4x4.Rotate(joint_displ);
					Matrix4x4 localTransform = Matrix4x4.TRS(link_vector, localRotationMatrix.rotation, new Vector3(1, 1, 1));
			
					segmentTransform = runningTransform * localTransform;

					break;
				}

			case ArticulationJointType.SphericalJoint:
				{
					// update direction according to current rotation
					Quaternion joint_displ = Quaternion.AngleAxis(delta_q_i, segment.jointIndex);
					Vector3 link_vector = segment.localPosition;

					// compound offset parent rotation with joint rotation:
					Matrix4x4 localRotationMatrix = Matrix4x4.Rotate(segment.localRotation) * Matrix4x4.Rotate(joint_displ);
					Matrix4x4 localTransform = Matrix4x4.TRS(Matrix4x4.Rotate(joint_displ) * link_vector, localRotationMatrix.rotation, new Vector3(1, 1, 1));
					segmentTransform = runningTransform * localTransform;
					break;
				}

			case ArticulationJointType.FixedJoint:
				{
					// fixed joint: update according only to the relative anchor
					Matrix4x4 localTransform = Matrix4x4.TRS(segment.localPosition+pVector, segment.localRotation, new Vector3(1, 1, 1));
					segmentTransform = runningTransform * localTransform;
					break;
				}

			default:
				{
					segmentTransform = runningTransform;
					break;
				}
		}
		return segmentTransform;
	}


	public Jacobian BuildJacobian(Vector<float> q_state, float dq = 1.0f)
	{

		/* Calculate the geometric Jacobian for the given manipulator
		 * 
		 * TODO: properly implement the KDL Jacobian method
		 * TODO: make sure this works for prismatic joints as well
		 * TODO: add constrained joint handling method
		 * TODO: Can we clean this up and make it more efficient? 
		 */

		// predeclare temporary variables
		Vector<float>[] JacArray = new Vector<float>[this.numberJoints];
		Vector<float> jacItem = Vector<float>.Build.Dense(6);

		// End effector position --> can no longer use segmentPositions shortcut
		Vector3 pEE = new Vector3(this.base2EETransform.m03, this.base2EETransform.m13, this.base2EETransform.m23); //this.segmentPositions[numberSegments - 1];

		int jj = 0;

		foreach (Segment seg in this.segments)
		{
			if (seg.joint.jointType != ArticulationJointType.FixedJoint)
			{
				// Generate full transformation matrix
				Matrix4x4 segmentTransform = Matrix4x4.TRS(segmentPositions[seg.index], segmentRotations[seg.index], new Vector3(1, 1, 1));

				// Linear velocity component
				Vector3 cartDelta = pEE - segmentPositions[seg.index]; // this will no longer be zero at final link - that's fine, I think.

				// 使用 Segment 中正确计算的 jointIndex（旋转轴）
				// 将局部轴变换到世界坐标系
				Vector3 localAxis = seg.jointIndex;
				Vector3 segAxis = segmentTransform.MultiplyVector(localAxis);
				
				// 确保轴是单位向量
				if (segAxis.magnitude > 0.001f)
				{
					segAxis = segAxis.normalized;
				}
				// Vector3 Jv = Vector3.Cross(cartDelta, segAxis); // “错误”的叉乘顺序
				Vector3 Jv = Vector3.Cross(segAxis, cartDelta);  // 注意：正确的叉乘顺序是 axis × r

				jacItem[0] = Jv.x;
				jacItem[1] = Jv.y;
				jacItem[2] = Jv.z;
				jacItem[3] = segAxis.x;
				jacItem[4] = segAxis.y;
				jacItem[5] = segAxis.z;

				// Condition vector elements
				for (int kk = 0; kk < jacItem.Count; kk++)
				{
					if (Mathf.Abs(jacItem[kk]) < jac_condition_eps) { jacItem[kk] = 0.0f; }
				}

				JacArray[jj] = Vector<float>.Build.Dense(6);
				jacItem.CopyTo(JacArray[jj]);
				jj++;
			}
		}

		return new Jacobian(JacArray);
	}

	// Below: trying the method described in Chembrammel and Kesavadas, implemented in KDL-orocos solver. Fast, but difficult to troubleshoot.
	/* Method: Move through the chain and calculate the change in segment position for a 1.0degree joint shift at each stage,
	* converting to segment frame and back iterating the segment transforms as we go. At the end, all Jacobian components will be in the
	* end-effector frame.
	* Use the principle axis of rotation to update the directions of the local segment deltas. */



	/*

		// Store jacobian as tuples of (vector, quaternion) (or whatever structure is most appropriate)
		// pass full (vector, quaternion) tuple list to jacobian constructor
		List<Vector3> jacCartList = new List<Vector3>();
		List<Quaternion> jacRotList = new List<Quaternion>();

		// The change in angle should always be dq, we can check this from axis-angle representation.
		// rotation elements should be dq*axisrepresentation = axis if d1 == 1.0

		// predeclare internal variables
		Matrix4x4 segOrigin = Matrix4x4.identity;
		Matrix4x4 segTransform = Matrix4x4.identity;
		Matrix4x4 deltaTransform = Matrix4x4.identity;

		// NOTE: by default baseTransform starts with the local (robot) coordinate system. To use world, replace with rootTransform.
		// Uh-oh. Where does the state matrix come in?


		int j = 0; // joint state vector iterator
		foreach (Segment seg in this.segments)
		{
			Vector3 dx_EE = new Vector3(0.0f, 0.0f, 0.0f);
			Quaternion dq_EE = Quaternion.identity;

			if (seg.joint.jointType != ArticulationJointType.FixedJoint)
			{
				// Calculate change in end-effector pose in local frame
				segOrigin = FwKinSegment(Matrix4x4.identity, seg, q_state[j]); // original segment pose in local origin
				segTransform = FwKinSegment(Matrix4x4.identity, seg, q_state[j]+dq); // new segment pose in local origin

				// Calculate partial derivative in end-effector frame
				dx_EE = new Vector3(segOrigin.m03 - segTransform.m03, segOrigin.m13 - segTransform.m13, segOrigin.m23 - segTransform.m23);
				dq_EE = Quaternion.Inverse(segTransform.rotation) * segOrigin.rotation;

				deltaTransform = Matrix4x4.TRS(new Vector3(0.0f, 0.0f, 0.0f), dq_EE, new Vector3(1, 1, 1));

			}
			else
			{
				// no movement (joint constraints can also be addressed here by calculating the appropriate segTransform)
				segOrigin = FwKinSegment(Matrix4x4.identity, seg, 0.0f);
				deltaTransform = Matrix4x4.identity;
			}

			Vector3[] jacCartBuild = new Vector3[j + 1];
			Quaternion[] jacRotBuild = new Quaternion[j + 1];

			// if fixed joint, we need to update the jacobian IF it already exists but we don't need to add a joint. 
			if (j > 0)
			{
				// copy lists to array
				jacCartBuild = jacCartList.ToArray();
				jacRotBuild = jacRotList.ToArray();

				// could put this into a subfunction
				// Update according to local segment transformation and any change in end-effector position
				for (int i = 0; i< jacCartBuild.Length; i++ )
				{
					Vector3 dx_update = deltaTransform * segOrigin.transpose * jacCartBuild[i];
					jacCartBuild[i] = dx_update;

					Matrix4x4 prevDelta = Matrix4x4.TRS(new Vector3(0.0f, 0.0f, 0.0f), jacRotBuild[i], new Vector3(1, 1, 1));
					segOrigin.SetColumn(3, new Vector4(0.0f, 0.0f, 0.0f, 1.0f));
					Quaternion drot_update = (deltaTransform * (segOrigin.transpose * prevDelta)).rotation;
					jacRotBuild[i] = drot_update;
				}
			}

			if (seg.joint.jointType != ArticulationJointType.FixedJoint)
			{
				// store new jacobian elements - copy array back to list then add new elements
				jacCartList.Clear();
				jacRotList.Clear();
				if (j > 0)
				{
					jacCartList.AddRange(jacCartBuild);
					jacRotList.AddRange(jacRotBuild);
				}
				jacCartList.Add(dx_EE);
				jacRotList.Add(dq_EE);
				j++;
			}
		}*/


}

