using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestVertexColor : MonoBehaviour
{
	[SerializeField] Color vertexColor = Color.white;
	[SerializeField] Color otherColor = Color.white;

	// Start is called before the first frame update
	void Start()
	{
		Mesh mesh = GetComponent<SkinnedMeshRenderer>().sharedMesh;
		Vector3[] vertices = mesh.vertices;

		// create new colors array where the colors will be created.
		Color[] colors = new Color[vertices.Length];

		for (int i = 0; i < vertices.Length; i++)
		{
			if (i < vertices.Length / 3)
				colors[i] = vertexColor;
			else
				colors[i] = otherColor;
		}

		// assign the array of colors to the Mesh.
		mesh.colors = colors;
	}
}
