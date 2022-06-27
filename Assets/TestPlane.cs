using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

public class TestPlane : MonoBehaviour
{
	[SerializeField] Transform a;
	[SerializeField] Transform b;
	[SerializeField] Transform c;

	[SerializeField] Transform obj;
	public void Update()
	{
		//var rotate = ExtensionVector.rollPitchYaw(a.localPosition, b.localPosition, c.localPosition);
	}

	void OnDrawGizmos()
	{
		var hipInfo = calcHips();

		var newRotate = Quaternion.Euler(hipInfo.Hips.rotation * Mathf.Rad2Deg);
		//Debug.Log($"{hipInfo.Hips.rotation * Mathf.Rad2Deg}");
		hipObj.localRotation = newRotate;

		calcPos();

	}

	void calcPos()
	{
		var aPos = a.position;
		var bPos = b.position;
		var cPos = c.position;

		var leftRotate = findRotationVector(bPos, cPos);
		leftRotate.x = 0.0f;
		//leftRotate.z = 0.0f;
		Handles.Label(bPos, $"{leftRotate * Mathf.Rad2Deg}");
		//leftRotate.x = ExtensionVector.angleBetween3DCoords(aPos, bPos, cPos);

		//var newRotate = Quaternion.Euler(leftRotate * Mathf.Rad2Deg);
		//obj.localRotation = newRotate;

		// var parentTrans = obj.parent ?? obj;
		// obj.rotation = Quaternion.identity;
		// obj.RotateAround(obj.position, parentTrans.right, leftRotate.x * Mathf.Rad2Deg);
		// obj.RotateAround(obj.position, parentTrans.up, leftRotate.y * Mathf.Rad2Deg);
		// obj.RotateAround(obj.position, parentTrans.forward, leftRotate.z * Mathf.Rad2Deg);

		// obj.localRotation = Quaternion.AngleAxis(leftRotate.z * Mathf.Rad2Deg, Vector3.forward) *
		// 					Quaternion.AngleAxis(leftRotate.y * Mathf.Rad2Deg, Vector3.up) *
		// 					Quaternion.AngleAxis(leftRotate.x * Mathf.Rad2Deg, Vector3.right);

	}

	static public Vector3 findRotationVector(Vector3 a, Vector3 b)
	{
		return new Vector3(
					find2DAngle(a.z, a.y, b.z, b.y),
					find2DAngle(a.x, -a.z, b.x, -b.z),
					find2DAngle(a.x, a.y, b.x, b.y)
				);
	}

	static public float find2DAngle(float cx, float cy, float ex, float ey)
	{
		float dy = ey - cy;
		float dx = ex - cx;
		if (dy == 0.0f && dx == 0.0f)
			return 0.0f;

		float theta = Mathf.Atan2(dy, dx);
		return theta;
	}

	void calcHead()
	{
		var aPos = a.position;
		var bPos = b.position;
		var cPos = c.position;

		var qb = bPos - aPos;
		var qc = cPos - aPos;
		var n = Vector3.Cross(qb, qc);

		var unitZ = n.normalized;
		var unitX = qb.normalized;
		var unitY = Vector3.Cross(unitZ, unitX);

		Gizmos.color = Color.green;
		Gizmos.DrawLine(aPos, aPos + unitY);
		Gizmos.color = Color.red;
		Gizmos.DrawLine(aPos, aPos + unitX);
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(aPos, aPos + unitZ);

		var beta = Mathf.Asin(-unitZ.x);               //좌/우로 (바라 보는 방향 변경).
		var alpha = Mathf.Atan2(unitZ.y, -unitZ.z);    //위/아래.
		var gamma = Mathf.Atan2(unitY.x, unitX.x);     //좌/우로 까딱까딱.

		var beta1 = ExtensionVector.normalizeAngle(beta);
		var alpha1 = ExtensionVector.normalizeAngle(alpha);
		var gamma1 = ExtensionVector.normalizeAngle(gamma);
		Handles.Label(cPos, $"{gamma * Mathf.Rad2Deg} -> {gamma1 * Mathf.Rad2Deg}");

		var newRotate = Quaternion.Euler(alpha1 * Mathf.Rad2Deg, beta1 * Mathf.Rad2Deg, gamma1 * Mathf.Rad2Deg);
		obj.localRotation = newRotate;// * Quaternion.Inverse(origRotate);
	}


	[SerializeField] Transform pos23;
	[SerializeField] Transform pos24;
	[SerializeField] Transform pos11;
	[SerializeField] Transform pos12;

	[SerializeField] Transform hipObj;
	public HipInfo calcHips()
	{
		//Find 2D normalized Hip and Shoulder Joint Positions/Distances
		Vector3 hipLeft2d = pos23.position;
		Vector3 hipRight2d = pos24.position;
		Vector3 shoulderLeft2d = pos11.position;
		Vector3 shoulderRight2d = pos12.position;
		var hipCenter2d = Vector3.Lerp(hipLeft2d, hipRight2d, 0.5f);
		var shoulderCenter2d = Vector3.Lerp(shoulderLeft2d, shoulderRight2d, 0.5f);
		var spineLength = (hipCenter2d - shoulderCenter2d).magnitude;

		var hips = new IHips()
		{
			position = new Vector3()
			{
				x = ExtensionVector.clamp(hipCenter2d.x - 0.4f, -1.0f, 1.0f), //subtract .4 to bring closer to 0,0 center
				y = 0.0f,
				z = ExtensionVector.clamp(spineLength - 1.0f, -2.0f, 0.0f),
			},
		};
		hips.worldPosition = new Vector3()
		{
			x = hips.position.x,
			y = 0.0f,
			z = hips.position.z * Mathf.Pow(hips.position.z * -2.0f, 2.0f),
		};
		hips.worldPosition.x *= hips.worldPosition.z;

		hips.rotation = ExtensionVector.rollPitchYaw(pos23.position, pos24.position);
		//fix -PI, PI jumping
		if (hips.rotation.y > 0.5f)
		{
			hips.rotation.y -= 2.0f;
		}
		hips.rotation.y += 0.5f;
		//Stop jumping between left and right shoulder tilt
		if (hips.rotation.z > 0.0f)
		{
			hips.rotation.z = 1.0f - hips.rotation.z;
		}
		if (hips.rotation.z < 0.0f)
		{
			hips.rotation.z = -1.0f - hips.rotation.z;
		}
		var turnAroundAmountHips = ExtensionVector.remap(Mathf.Abs(hips.rotation.y), 0.2f, 0.4f);
		hips.rotation.z *= 1.0f - turnAroundAmountHips;
		hips.rotation.x = 0.0f; //temp fix for inaccurate X axis

		var spine = ExtensionVector.rollPitchYaw(pos11.position, pos12.position);
		//fix -PI, PI jumping
		if (spine.y > 0.5f)
		{
			spine.y -= 2.0f;
		}
		spine.y += 0.5f;
		//Stop jumping between left and right shoulder tilt
		if (spine.z > 0.0f)
		{
			spine.z = 1.0f - spine.z;
		}
		if (spine.z < 0.0f)
		{
			spine.z = -1.0f - spine.z;
		}
		//fix weird large numbers when 2 shoulder points get too close
		var turnAroundAmount = ExtensionVector.remap(Mathf.Abs(spine.y), 0.2f, 0.4f);
		spine.z *= 1.0f - turnAroundAmount;
		spine.x = 0.0f; //temp fix for inaccurate X axis

		return rigHips(hips, spine);
	}

	HipInfo rigHips(IHips hips, Vector3 spine)
	{
		//convert normalized values to radians
		if (hips.rotation != null)
		{
			hips.rotation.x *= Mathf.PI;
			hips.rotation.y *= Mathf.PI;
			hips.rotation.z *= Mathf.PI;
		}

		spine.x *= Mathf.PI;
		spine.y *= Mathf.PI;
		spine.z *= Mathf.PI;

		return new HipInfo()
		{
			Hips = hips,
			Spine = spine,
		};
	}
}
