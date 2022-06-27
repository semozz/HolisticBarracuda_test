using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Line : MonoBehaviour
{
	[SerializeField] Transform from;
	[SerializeField] Transform to;

	LineRenderer lineRender;
	private void Start()
	{
		lineRender = GetComponent<LineRenderer>();
	}

	private void Update()
	{
		var fromPos = from.position;
		var toPos = to.position;

		if (from.gameObject.active == false ||
			to.gameObject.active == false)
		{
			lineRender.startWidth = 0.0f;
			lineRender.endWidth = 0.0f;
		}
		else
		{
			lineRender.startWidth = 0.01f;
			lineRender.endWidth = 0.01f;
		}

		lineRender.SetPosition(0, fromPos);
		lineRender.SetPosition(1, toPos);
	}
}
