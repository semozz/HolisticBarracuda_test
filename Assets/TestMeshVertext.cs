using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestMeshVertext : MonoBehaviour
{
	[SerializeField] GameObject effectObj;
	[SerializeField] int vertextIndex = 0;
	[SerializeField] SkinnedMeshRenderer faceMesh;

	private void LateUpdate()
	{
		var startTime = System.DateTime.Now;
		Mesh baked = new Mesh();
		faceMesh.BakeMesh(baked);

		Transform myTrans = this.transform;
		Vector3 worldPos = myTrans.localToWorldMatrix.MultiplyPoint3x4(baked.vertices[vertextIndex]);

		//Debug.Log(worldPos);
		effectObj.transform.position = worldPos;

		var endTime = System.DateTime.Now;

		Debug.Log($"Time : {endTime - startTime}");
	}

	private static void CopyBlendShapes(Mesh originalMesh, Mesh newMesh)
	{
		for (int i = 0; i < originalMesh.blendShapeCount; i++)
		{
			string shapeName = originalMesh.GetBlendShapeName(i);
			int frameCount = originalMesh.GetBlendShapeFrameCount(i);
			for (int j = 0; j < frameCount; j++)
			{
				Vector3[] dv = new Vector3[originalMesh.vertexCount];
				Vector3[] dn = new Vector3[originalMesh.vertexCount];
				Vector3[] dt = new Vector3[originalMesh.vertexCount];

				float frameWeight = originalMesh.GetBlendShapeFrameWeight(i, j);
				originalMesh.GetBlendShapeFrameVertices(i, j, dv, dn, dt);
				newMesh.AddBlendShapeFrame(shapeName, frameWeight, dv, dn, dt);
			}
		}
	}
}
