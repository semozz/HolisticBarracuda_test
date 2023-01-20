using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.U2D;

[System.Serializable]
public class BoneInfo
{
	[SerializeField] public Transform bone;
	[SerializeField] public int fromIndex;
	[SerializeField] public int toIndex;

	[SerializeField] public Transform fromNode;
	[SerializeField] public Transform toNode;

	[SerializeField] public bool isFoot = false;
	[SerializeField] public UnityEngine.Vector3 origDir = UnityEngine.Vector3.up;
	[SerializeField] public UnityEngine.Quaternion origiRotate = UnityEngine.Quaternion.identity;
}

[System.Serializable]
public class Skelleton : MonoBehaviour
{
	public List<GameObject> nodeList = new();

	public List<BoneInfo> boneList = new List<BoneInfo>();
	public bool useLateUpdate = false;

	private void Awake()
	{
		foreach (var bone in boneList)
		{
			bone.origiRotate = bone.bone.rotation;
		}


	}
	private void LateUpdate()
	{
		if (useLateUpdate == false)
			return;

		foreach (var bone in boneList)
		{
			//if (bone.isFoot == false)
			//	continue;

			var fromPos = bone.fromNode.position;
			var toPos = bone.toNode.position;

			var dir = (toPos - fromPos).normalized;
			var origRotate = bone.origDir != UnityEngine.Vector3.zero ? UnityEngine.Quaternion.LookRotation(bone.origDir.normalized) : UnityEngine.Quaternion.identity;

			UnityEngine.Quaternion result = UnityEngine.Quaternion.identity;
			if (bone.isFoot == true)
			{
				result = UnityEngine.Quaternion.LookRotation(dir) * UnityEngine.Quaternion.Inverse(origRotate);
				bone.bone.rotation = result;//
											// UnityEngine.Quaternion.Lerp(bone.bone.localRotation,
											// 										result,
											// 										0.8f);
			}
			else
			{
				result = UnityEngine.Quaternion.LookRotation(dir) * UnityEngine.Quaternion.Inverse(origRotate);


				bone.bone.rotation = UnityEngine.Quaternion.Lerp(bone.bone.rotation,
														result,
														0.8f);
			}


		}
	}

	public void UpdatePos(Vector4[] posList, Vector4[] faceList)
	{
		int index = 0;
		foreach (var obj in nodeList)
		{
			var info = posList[index];
			obj.SetActive(info.w != 0.0f);

			obj.transform.localPosition = GetPos(index, posList);
			index++;
		}

		foreach (var bone in boneList)
		{
			var fromPos = GetPos(bone.fromIndex, posList);
			var toPos = GetPos(bone.toIndex, posList);
			var dir = (toPos - fromPos).normalized;

			var origRotate = bone.origDir != Vector3.zero ? Quaternion.LookRotation(bone.origDir) : Quaternion.identity;
			var result = Quaternion.LookRotation(dir) * Quaternion.Inverse(origRotate);

			if (bone.fromIndex == 12)
			{
				var deg = result.eulerAngles;
				Debug.Log($"1 : RightUpperArm : {deg.x}, {deg.y}, {deg.z}");
			}

			bone.bone.rotation = Quaternion.Lerp(bone.bone.rotation,
														result,
														0.8f);

			if (bone.fromIndex == 12)
			{
				var deg = bone.bone.localRotation.eulerAngles;
				Debug.Log($"1-1 : RightUpperArm : {deg.x}, {deg.y}, {deg.z}");
			}
		}

		var hipsInfo = MotionTracking.calcHips(posList);


		var riggedLegs = MotionTracking.calcLegs(posList);



		// var riggedPose = MotionTracking.calcArms2(posList);
		// RotateBone(HumanBodyBones.RightUpperArm, riggedPose.unscaledUpperArm.rightRot);
		// RotateBone(HumanBodyBones.RightLowerArm, riggedPose.unscaledLowerArm.rightRot);
		// RotateBone(HumanBodyBones.LeftUpperArm, riggedPose.unscaledUpperArm.leftRot);
		// RotateBone(HumanBodyBones.LeftLowerArm, riggedPose.unscaledLowerArm.leftRot);

		var riggedPose = MotionTracking.calcArms(posList);
		var axisMap = new AxisMap() { x = 2, y = 0, z = 1 };
		var rotationVector = ExtensionVector.transAxis(riggedPose.UpperArm.right, axisMap);
		LocalRotateBone(HumanBodyBones.RightUpperArm, rotationVector);
		// LocalRotateBone(HumanBodyBones.RightLowerArm, riggedPose.LowerArm.right);
		//LocalRotateBone(HumanBodyBones.LeftUpperArm, riggedPose.UpperArm.left);
		// LocalRotateBone(HumanBodyBones.LeftLowerArm, riggedPose.LowerArm.left);

		LocalRotateBone(HumanBodyBones.Hips, hipsInfo.Hips.rotation);

		//RotateBone(HumanBodyBones.RightUpperLeg, riggedLegs.unscaledUpperLeg.right);
		// RotateBone(HumanBodyBones.RightLowerLeg, riggedLegs.LowerLeg.right);
		//RotateBone(HumanBodyBones.LeftLowerLeg, riggedLegs.unscaledUpperLeg.left);
		// RotateBone(HumanBodyBones.LeftLowerLeg, riggedLegs.LowerLeg.left);

		ApplyFaceInfo(faceList);

	}

	void ApplyFaceInfo(Vector4[] faceList)
	{
		var headPlane = MotionTracking.calcHead(faceList);
		var mouthInfo = MotionTracking.calcMouth(faceList);
		var eyeInfo = MotionTracking.calcEyes(faceList);
		eyeInfo = MotionTracking.stabilizeBlink(eyeInfo, headPlane.y);
		var pupils = MotionTracking.calcPupils(faceList);
		var brow = MotionTracking.calcBrow(faceList);

		Debug.Log($"I : {mouthInfo.shape.I}, A : {mouthInfo.shape.A}, E : {mouthInfo.shape.E}, O : {mouthInfo.shape.O}, U : {mouthInfo.shape.U}");

		//LocalRotateBone(HumanBodyBones.Neck, headPlane.x, headPlane.y, headPlane.z);

		rigFace(headPlane, mouthInfo, eyeInfo, pupils, brow);

	}

	[SerializeField] SkinnedMeshRenderer faceRenderer;
	string blendShapePrefix = "";
	void rigFace(HeadPlane head, MouthInfo mouth, EyeInfo eye, PupilPos pupil, float brow)
	{
		if (blendShapeKeyValues.Count == 0)
			InitBlendShapeKeyValues();

		SetBlendShapeValue($"{blendShapePrefix}.eyeBlinkLeft", eye.left);
		SetBlendShapeValue($"{blendShapePrefix}.eyeBlinkRight", eye.right);
	}

	void SetBlendShapeValue(string key, float weight)
	{
		if (faceRenderer == null)
			return;

		int index = -1;
		if (blendShapeValues.ContainsKey(key))
			index = blendShapeValues[key];

		if (index == -1)
			return;

		var frame = blendShapeFrames[index];
		// var value = (int)((float)frame * weight);
		var targetValue = (1.0f - weight) * 100.0f;
		var curValue = faceRenderer.GetBlendShapeWeight(index);
		var value = Mathf.Lerp(curValue, targetValue, 0.8f);
		faceRenderer.SetBlendShapeWeight(index, value);

		Debug.Log($"{key} - {frame}, {value}");
	}

	Dictionary<int, string> blendShapeKeyValues = new Dictionary<int, string>();
	Dictionary<int, int> blendShapeFrames = new Dictionary<int, int>();
	Dictionary<string, int> blendShapeValues = new Dictionary<string, int>();
	void InitBlendShapeKeyValues()
	{
		if (faceRenderer == null)
			return;

		var faceMesh = faceRenderer.sharedMesh;

		blendShapeKeyValues.Clear();
		int count = faceMesh.blendShapeCount;
		for (int i = 0; i < count; ++i)
		{
			var name = faceMesh.GetBlendShapeName(i);
			var frame = faceMesh.GetBlendShapeFrameCount(i);
			blendShapeKeyValues.Add(i, name);
			blendShapeFrames.Add(i, frame);
			blendShapeValues.Add(name, i);

			if (string.IsNullOrEmpty(blendShapePrefix) == true)
			{
				var tokens = name.Split('.');
				blendShapePrefix = tokens[0];
			}

			Debug.Log($"{i} -> {name}");
		}
	}




	public Vector3 GetAngle(Vector3 from, Vector3 to, bool reverse)
	{
		var result = Vector3.zero;

		Vector3 v = (to - from).normalized;
		if (reverse == false)
		{
			//위/아래 회전.
			result.x = Mathf.Atan2(-v.y, v.x) * Mathf.Rad2Deg;
			//앞/뒤 회전.
			result.z = Mathf.Atan2(-v.z, v.x) * Mathf.Rad2Deg;
			//
			//result.y = Mathf.Atan2(v.z, -v.y) * Mathf.Rad2Deg;
		}
		else
		{
			result.x = Mathf.Atan2(v.y, v.x) * Mathf.Rad2Deg;
			result.z = Mathf.Atan2(-v.z, -v.x) * Mathf.Rad2Deg;
			//result.x = Mathf.Atan2(v.z, v.y) * Mathf.Rad2Deg;
		}

		return result;
	}

	Vector3 GetPos(int index, Vector4[] posList)
	{
		Vector3 pos = Vector3.zero;
		if (index >= 0 && index < posList.Length)
		{
			var t = posList[index];
			pos.x = t.x;
			pos.y = t.y;
			pos.z = t.z;

			//pos *= 5.0f;
		}

		return pos;
	}


	[SerializeField] Transform a;
	[SerializeField] Transform b;
	[SerializeField] Transform c;

	void OnDrawGizmos()
	{
		foreach (var bone in boneList)
		{
			if (bone.fromNode == null || bone.toNode == null)
				continue;

			var startPos = bone.fromNode.position;
			var endPos = bone.toNode.position;


			var forward = endPos - startPos;//bone.bone.TransformDirection(Vector3.forward);
			var right = bone.bone.TransformDirection(UnityEngine.Vector3.right);
			var up = -UnityEngine.Vector3.Cross(forward, right);

			Gizmos.DrawLine(startPos, endPos);

			Gizmos.color = Color.red;
			Gizmos.DrawLine(startPos, startPos + forward);
			Gizmos.color = Color.green;
			Gizmos.DrawLine(startPos, startPos + right);
			Gizmos.color = Color.blue;
			Gizmos.DrawLine(startPos, startPos + up);
		}

		if (a == null || b == null || c == null)
			return;

		// Vector3 dir = c.position - b.position;
		// var orig = Quaternion.LookRotation(Vector3.up);
		// var nextRot = Quaternion.LookRotation(dir) * Quaternion.Inverse(orig);

		// RotateBone(HumanBodyBones.LeftUpperArm, nextRot);

	}

	void RotateBone(HumanBodyBones bone, Quaternion targetRot)
	{
		if (humanoid == null || humanoidAnimator == null)
			return;

		var Part = humanoidAnimator.GetBoneTransform(bone);
		if (Part == null)
			return;

		Part.rotation = Quaternion.Lerp(Part.rotation, targetRot, 0.8f);
	}

	void RotateBone(HumanBodyBones bone, Vector3 targetRot)
	{
		var rotate = Quaternion.Euler(targetRot * Mathf.Rad2Deg);
		RotateBone(bone, rotate);
	}

	void RotateBone(HumanBodyBones bone, float x, float y, float z)
	{
		RotateBone(bone, new Vector3(x, y, z));
	}

	void LocalRotateBone(HumanBodyBones bone, float x, float y, float z)
	{
		LocalRotateBone(bone, new Vector3(x, y, z));
	}

	void LocalRotateBone(HumanBodyBones bone, Vector3 angle)
	{
		if (humanoid == null || humanoidAnimator == null)
			return;

		var Part = humanoidAnimator.GetBoneTransform(bone);
		if (Part == null)
			return;

		var rotate = Quaternion.Euler(angle * Mathf.Rad2Deg);
		Part.localRotation = Quaternion.Lerp(Part.localRotation, rotate, 1.0f);
	}

	void rigRotation(HumanBodyBones bone, Vector3 rotation, float dampener = 1.0f, float lerpAmount = 0.3f)
	{
		if (humanoid == null || humanoidAnimator == null)
			return;

		var Part = humanoidAnimator.GetBoneTransform(bone);
		if (Part == null)
			return;

		// Quaternion rot = Part.rotation;

		// var quaternion = Quaternion.Inverse(rot) * Quaternion.Euler(rotation * Mathf.Rad2Deg * dampener);
		// Part.rotation = Quaternion.Slerp(Part.rotation, quaternion, lerpAmount); // interpolate

		var eulerAngle = (rotation * Mathf.Rad2Deg * dampener);
		Part.Rotate(eulerAngle, Space.Self);
	}

	void blendShapeSetValue(SkinnedMeshRenderer renderer, int key, float value)
	{
		Mesh mesh = renderer.sharedMesh;
		var count = mesh.blendShapeCount;
		for (int i = 0; i < count; ++i)
		{
			string name = mesh.GetBlendShapeName(i);
			int index = mesh.GetBlendShapeIndex(name);

		}
		renderer.SetBlendShapeWeight(key, value);
	}


	[SerializeField] Transform humanoid;
	[SerializeField] Animator humanoidAnimator;


}



