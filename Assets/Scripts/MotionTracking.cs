using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;



struct BodyParts
{
	public Vector3 right;
	public Vector3 left;

	public Quaternion rightRot;
	public Quaternion leftRot;
}

struct RigArms
{
	public Vector3 UpperArm;
	public Vector3 LowerArm;
	public Vector3 Hand;

}

struct ArmsInfo
{
	public BodyParts UpperArm;
	public BodyParts LowerArm;
	public BodyParts Hand;

	public BodyParts unscaledUpperArm;
	public BodyParts unscaledLowerArm;
	public BodyParts unscaledHand;

}

struct RigLeg
{
	public Vector3 UpperLeg;
	public Vector3 LowerLeg;
}

struct LegsInfo
{
	public BodyParts UpperLeg;
	public BodyParts LowerLeg;

	public BodyParts unscaledUpperLeg;
	public BodyParts unscaledLowerLeg;
}

struct AxisMap
{
	public int x;
	public int y;
	public int z;

	static public AxisMap defaultValue()
	{
		return new AxisMap() { x = 0, y = 1, z = 2 };
	}
}

struct SphericalCoords
{
	public float theta;
	public float phi;
}

struct EulerPlane
{
	public Vector3[] vector;
	public Vector3[] points;
}

struct HeadPlane
{
	public float x;
	public float y;
	public float z;
	public float width;
	public float height;
	public Vector3 position;
	public Vector3 normalized;
	public Vector3 degrees;
}

struct MouthInfo
{
	public float x;
	public float y;

	public MouthShape shape;
}

struct MouthShape
{
	public float A;
	public float E;
	public float I;
	public float O;
	public float U;
}

struct EyeOpen
{
	public float norm;
	public float raw;
}

struct EyeInfo
{
	public float left;
	public float right;
}

struct PupilPos
{
	public float x;
	public float y;
}

public struct IHips
{
	public Vector3 position;
	public Vector3 rotation;
	public Vector3 worldPosition;
}

public struct HipInfo
{
	public IHips Hips;
	public Vector3 Spine;
}

enum Side
{
	Right,
	Left
}

class Points
{
	static public int[][] eye = new int[][]
		{
			new int[] {263, 362, 387, 386, 385, 373, 374, 380}, //RIGHT
			new int[] {130, 133, 160, 159, 158, 144, 145, 153}, //LEFT
		};


	static public int[][] brow = new int[][] {
			new int[] {265, 464, 293, 334, 296, 449, 450, 451}, //RIGHT
			new int[] {35, 244, 63, 105, 66, 229, 230, 231}, //LEFT
		};
	static public int[][] pupil = new int[][] {
			new int[] {473, 474, 475, 476, 477},	//RIGHT
			new int[] {468, 469, 470, 471, 472}, //LEFT
		};
}

class ExtensionVector
{
	static public AxisMap defaultAxis = new AxisMap() { x = 0, y = 1, z = 2 };

	static public Vector3 transAxis(Vector3 orig, AxisMap axis)
	{
		return new Vector3()
		{
			x = orig[axis.x],
			y = orig[axis.y],
			z = orig[axis.z]
		};
	}
	static public Vector3 findRotationVector(Vector3 a, Vector3 b)
	{
		return new Vector3(
					find2DAngle(a.z, a.y, b.z, b.y),
					find2DAngle(a.x, -a.z, b.x, -b.z),
					find2DAngle(a.x, a.y, b.x, b.y)
				);
	}
	static public Vector3 findRotation(Vector3 a, Vector3 b, bool normalize = true)
	{
		var rotation = findRotationVector(a, b);
		if (normalize)
		{
			rotation.x = normalizeRadians(rotation.x);
			rotation.y = normalizeRadians(rotation.y);
			rotation.z = normalizeRadians(rotation.z);
		}

		return rotation;
	}

	static public Quaternion findRotation2(Vector3 a, Vector3 b)
	{
		var dir = (b - a).normalized;

		var origRotate = UnityEngine.Quaternion.LookRotation(Vector3.up);
		var result = UnityEngine.Quaternion.LookRotation(dir) * UnityEngine.Quaternion.Inverse(origRotate);

		return result;
	}

	static public float find2DAngle(float cx, float cy, float ex, float ey)
	{
		float dy = ey - cy;
		float dx = ex - cx;
		float theta = Mathf.Atan2(dy, dx);
		return theta;
	}

	static public float normalizeAngle(float radians)
	{
		var TWO_PI = Mathf.PI * 2.0f;

		var angle = radians % TWO_PI;
		angle = angle > Mathf.PI ? angle - TWO_PI : angle < -Mathf.PI ? TWO_PI + angle : angle;
		//returns normalized values to -1,1
		return angle / Mathf.PI;
	}

	static public float normalizeRadians(float radians)
	{
		if (radians >= Mathf.PI / 2)
		{
			radians -= Mathf.PI * 2;
		}
		if (radians <= -Mathf.PI / 2)
		{
			radians += Mathf.PI * 2;
			radians = Mathf.PI - radians;
		}
		//returns normalized values to -1,1
		return radians / Mathf.PI;
	}

	static public float angleBetween3DCoords(Vector3 a, Vector3 b, Vector3 c)
	{
		// Calculate vector between points 1 and 2
		var v1 = a - b;

		// Calculate vector between points 2 and 3
		var v2 = c - b;

		// The dot product of vectors v1 & v2 is a function of the cosine of the
		// angle between them (it's scaled by the product of their magnitudes).
		var v1norm = v1.normalized;
		var v2norm = v2.normalized;

		// Calculate the dot products of vectors v1 and v2
		var dotProducts = Vector3.Dot(v1norm, v2norm);

		var cross = Vector3.Cross(v1norm, v2norm);

		// Extract the angle from the dot products
		float angle = Mathf.Acos(dotProducts);
		if (cross == Vector3.zero)
			angle = Mathf.PI;

		if (cross.z < 0.0f)
			angle = (Mathf.PI * 2.0f) - angle;


		// Gizmos.DrawLine(a, b);
		// Gizmos.DrawLine(b, c);
		// UnityEditor.Handles.Label(b, $"{angle * Mathf.Rad2Deg}, {cross}");

		// return single angle Normalized to 1
		return angle;//normalizeRadians(angle);
	}


	/**
     * Get normalized, spherical coordinates for the vector bc, relative to vector ab
     * @param {Vector | number} a: Vector or Number
     * @param {Vector | number} b: Vector or Number
     * @param {Vector | number} c: Vector or Number
     * @param {AxisMap} axisMap: Mapped axis to get the right spherical coords
     */
	static public SphericalCoords getRelativeSphericalCoords(Vector3 a, Vector3 b, Vector3 c, AxisMap axisMap)
	{

		// Calculate vector between points 1 and 2
		var v1 = b - a;

		// Calculate vector between points 2 and 3
		var v2 = c - b;

		var v1norm = v1.normalized;
		var v2norm = v2.normalized;


		var coord1 = toSphericalCoords(v1norm, axisMap);
		var coord2 = toSphericalCoords(v2norm, axisMap);

		var theta = coord1.theta - coord2.theta;
		var phi = coord1.phi - coord2.phi;

		return new SphericalCoords()
		{
			theta = normalizeAngle(theta),
			phi = normalizeAngle(phi),
		};
	}

	/**
     * Get normalized, spherical coordinates for the vector bc
     * @param {Vector | number} a: Vector or Number
     * @param {Vector | number} b: Vector or Number
     * @param {AxisMap} axisMap: Mapped axis to get the right spherical coords
     */
	static public SphericalCoords getSphericalCoords(Vector3 a, Vector3 b, AxisMap axisMap)
	{

		// Calculate vector between points 1 and 2
		var v1 = b - a;

		var v1norm = v1.normalized;
		var coord = toSphericalCoords(v1norm, axisMap);

		return new SphericalCoords()
		{
			theta = normalizeAngle(-coord.theta),
			phi = normalizeAngle((Mathf.PI / 2.0f) - coord.phi),
		};
	}

	/**
     * To Angles
     * @param {AxisMap} [axisMap = {x: "x", y: "y", z: "z"}]
     * @returns {{ theta: number, phi: number }}
     */
	static public SphericalCoords toSphericalCoords(Vector3 a, AxisMap axisMap)
	{
		return new SphericalCoords()
		{
			theta = Mathf.Atan2(a[axisMap.y], a[axisMap.x]),
			phi = Mathf.Acos(a[axisMap.z] / a.magnitude),
		};
	}

	static public Vector3 rollPitchYaw(Vector3 a, Vector3 b)
	{
		return new Vector3(
				normalizeAngle(find2DAngle(a.z, a.y, b.z, b.y)),
				normalizeAngle(find2DAngle(a.z, a.x, b.z, b.x)),
				normalizeAngle(find2DAngle(a.x, -a.y, b.x, -b.y))
			);
	}
	static public Vector3 rollPitchYaw(Vector3 a, Vector3 b, Vector3 c)
	{
		var qb = b - a;
		var qc = c - a;
		var n = Vector3.Cross(qb, qc);

		var unitZ = n.normalized;
		var unitX = qb.normalized;
		var unitY = Vector3.Cross(unitZ, unitX);

		// var beta = Mathf.Asin(unitZ.x);
		// var alpha = Mathf.Atan2(unitZ.y, unitZ.z);
		// var gamma = Mathf.Atan2(-unitY.x, unitX.x);

		var beta = Mathf.Asin(-unitZ.x);               //좌/우로 (바라 보는 방향 변경).
		var alpha = Mathf.Atan2(unitZ.y, -unitZ.z);    //위/아래.
		var gamma = Mathf.Atan2(unitY.x, unitX.x);     //좌/우로 까딱까딱.

		return new Vector3(normalizeAngle(alpha), normalizeAngle(beta), normalizeAngle(gamma));
	}

	/**
	* Returns a clamped value between min and max values
	* @param {Number} val : transformed value
	* @param {Number} min : minimum value
	* @param {Number} max : maximum value
	*/
	static public float clamp(float val, float min, float max)
	{
		return Mathf.Max(Mathf.Min(val, max), min);
	}

	/**
	* Returns a remapped value between 0 and 1 using min and max values
	* @param {Number} value : transformed value
	* @param {Number} min : minimum value
	* @param {Number} max : maximum value
	*/
	static public float remap(float val, float min, float max)
	{
		//returns min to max -> 0 to 1
		return (clamp(val, min, max) - min) / (max - min);
	}
}

class MotionTracking
{
	static public ArmsInfo calcArms(Vector4[] lm)
	{
		BodyParts UpperArm = new BodyParts()
		{
			left = ExtensionVector.findRotation(lm[11], lm[13], false),
			right = ExtensionVector.findRotation(lm[12], lm[14], false),
		};
		UpperArm.left.x = 0.0f;//ExtensionVector.angleBetween3DCoords(lm[12], lm[11], lm[13]);
		UpperArm.right.x = 0.0f;//ExtensionVector.angleBetween3DCoords(lm[11], lm[12], lm[14]);

		BodyParts LowerArm = new BodyParts()
		{
			left = ExtensionVector.findRotation(lm[13], lm[15], false),
			right = ExtensionVector.findRotation(lm[14], lm[16], false),
		};
		LowerArm.left.x = 0.0f;//ExtensionVector.angleBetween3DCoords(lm[11], lm[13], lm[15]);
		LowerArm.right.x = 0.0f;//ExtensionVector.angleBetween3DCoords(lm[12], lm[14], lm[16]);
								//LowerArm.right.z = Mathf.Clamp(LowerArm.right.z, -2.14f, 0);
								//LowerArm.left.z = Mathf.Clamp(LowerArm.left.z, -2.14f, 0);

		BodyParts Hand = new BodyParts()
		{
			left = ExtensionVector.findRotation(
				lm[15],
				Vector3.Lerp(lm[17], lm[19], 0.5f), false
			),
			right = ExtensionVector.findRotation(
				lm[16],
				Vector3.Lerp(lm[18], lm[20], 0.5f), false
			),
		};

		//Modify Rotations slightly for more natural movement
		var rightArmRig = rigArm(UpperArm.right, LowerArm.right, Hand.right, Side.Right);
		var leftArmRig = rigArm(UpperArm.left, LowerArm.left, Hand.left, Side.Left);

		var deg = UpperArm.right * Mathf.Rad2Deg;
		Debug.Log($"2 : RightUpperArm : {deg.x}, {deg.y}, {deg.z}");
		return new ArmsInfo()
		{
			UpperArm = new BodyParts()
			{
				right = rightArmRig.UpperArm,
				left = leftArmRig.UpperArm
			},
			LowerArm = new BodyParts()
			{
				right = rightArmRig.LowerArm,
				left = leftArmRig.LowerArm
			},
			Hand = new BodyParts()
			{
				right = rightArmRig.Hand,
				left = leftArmRig.Hand,
			},

			unscaledUpperArm = UpperArm,
			unscaledLowerArm = LowerArm,
			unscaledHand = Hand
		};
	}

	static public ArmsInfo calcArms2(Vector4[] lm)
	{
		BodyParts UpperArm = new BodyParts()
		{
			rightRot = ExtensionVector.findRotation2(lm[12], lm[14]),
			leftRot = ExtensionVector.findRotation2(lm[11], lm[13]),
		};

		BodyParts LowerArm = new BodyParts()
		{
			rightRot = ExtensionVector.findRotation2(lm[14], lm[16]),
			leftRot = ExtensionVector.findRotation2(lm[13], lm[15]),
		};

		BodyParts Hand = new BodyParts()
		{
			rightRot = ExtensionVector.findRotation2(
				lm[16],
				Vector3.Lerp(lm[18], lm[20], 0.5f)
			),
			leftRot = ExtensionVector.findRotation2(
				lm[15],
				Vector3.Lerp(lm[17], lm[19], 0.5f)
			),
		};

		//Modify Rotations slightly for more natural movement
		//var rightArmRig = rigArm(UpperArm.right, LowerArm.right, Hand.right, Side.Right);
		//var leftArmRig = rigArm(UpperArm.left, LowerArm.left, Hand.left, Side.Left);

		var deg = UpperArm.rightRot.eulerAngles;// * Mathf.Rad2Deg;
		Debug.Log($"2 : RightUpperArm : {deg.x}, {deg.y}, {deg.z}");
		return new ArmsInfo()
		{
			UpperArm = new BodyParts()
			{
				right = UpperArm.right,
				left = UpperArm.left
			},
			LowerArm = new BodyParts()
			{
				right = LowerArm.right,
				left = LowerArm.left
			},
			Hand = new BodyParts()
			{
				right = Hand.right,
				left = Hand.left,
			},

			unscaledUpperArm = UpperArm,
			unscaledLowerArm = LowerArm,
			unscaledHand = Hand
		};
	}

	/**
	* Converts normalized rotation values into radians clamped by human limits
	* @param {Object} UpperArm : normalized rotation values
	* @param {Object} LowerArm : normalized rotation values
	* @param {Object} Hand : normalized rotation values
	* @param {Side} side : left or right
	*/
	static public RigArms rigArm(Vector3 UpperArm, Vector3 LowerArm, Vector3 Hand, Side side = Side.Right)
	{
		/*
		// Invert modifier based on left vs right side
		var invert = side == Side.Right ? 1.0f : -1.0f;

		UpperArm.z *= -2.3f * invert;
		//Modify UpperArm rotationY  by LowerArm X and Z rotations
		UpperArm.y *= Mathf.PI * invert;
		UpperArm.y -= Mathf.Max(LowerArm.x);
		UpperArm.y -= -invert * Mathf.Max(LowerArm.z, 0.0f);
		UpperArm.x -= 0.3f * invert;

		LowerArm.z *= -2.14f * invert;
		LowerArm.y *= 2.14f * invert;
		LowerArm.x *= 2.14f * invert;

		//Clamp values to human limits
		UpperArm.x = Mathf.Clamp(UpperArm.x, -0.5f, Mathf.PI);
		LowerArm.x = Mathf.Clamp(LowerArm.x, -0.3f, 0.3f);

		Hand.y = Mathf.Clamp(Hand.z * 2, -0.6f, 0.6f); //side to side
		Hand.z = Hand.z * -2.3f * invert; //up down
		*/
		return new RigArms()
		{
			//Returns Values in Radians for direct 3D usage
			UpperArm = UpperArm,
			LowerArm = LowerArm,
			Hand = Hand,
		};
	}



	/**
	* Calculates leg rotation angles
	* @param {Results} lm : array of 3D pose vectors from tfjs or mediapipe
	*/
	static public LegsInfo calcLegs(Vector4[] lm)
	{
		var rightUpperLegSphericalCoords =
						ExtensionVector.getSphericalCoords(lm[24], lm[26], new AxisMap() { x = 1, y = 2, z = 0 });
		var leftUpperLegSphericalCoords =
						ExtensionVector.getSphericalCoords(lm[23], lm[25], new AxisMap() { x = 1, y = 2, z = 0 });
		var rightLowerLegSphericalCoords =
						ExtensionVector.getRelativeSphericalCoords(lm[24], lm[26], lm[28], new AxisMap() { x = 1, y = 2, z = 0, });
		var leftLowerLegSphericalCoords =
						ExtensionVector.getRelativeSphericalCoords(lm[23], lm[25], lm[27], new AxisMap() { x = 1, y = 2, z = 0, });
		var hipRotation = ExtensionVector.findRotation(lm[23], lm[24]);

		var UpperLeg = new BodyParts()
		{
			right = new Vector3()
			{
				x = rightUpperLegSphericalCoords.theta,
				y = rightLowerLegSphericalCoords.phi,
				z = rightUpperLegSphericalCoords.phi - hipRotation.z,
			},
			left = new Vector3()
			{
				x = leftUpperLegSphericalCoords.theta,
				y = leftLowerLegSphericalCoords.phi,
				z = leftUpperLegSphericalCoords.phi - hipRotation.z,
			},
		};

		var LowerLeg = new BodyParts()
		{
			right = new Vector3()
			{
				x = -Mathf.Abs(rightLowerLegSphericalCoords.theta),
				y = 0.0f, // not relevant
				z = 0.0f, // not relevant
			},
			left = new Vector3()
			{
				x = -Mathf.Abs(leftLowerLegSphericalCoords.theta),
				y = 0.0f, // not relevant
				z = 0.0f, // not relevant
			},
		};

		var rightLegRig = rigLeg(UpperLeg.right, LowerLeg.right, Side.Right);
		var leftLegRig = rigLeg(UpperLeg.left, LowerLeg.left, Side.Left);

		return new LegsInfo()
		{
			UpperLeg = new BodyParts()
			{
				right = rightLegRig.UpperLeg,
				left = leftLegRig.UpperLeg,
			},
			LowerLeg = new BodyParts()
			{
				right = rightLegRig.LowerLeg,
				left = leftLegRig.LowerLeg,
			},

			unscaledUpperLeg = UpperLeg,
			unscaledLowerLeg = LowerLeg,
		};
	}

	/**
	* Converts normalized rotation values into radians clamped by human limits
	* @param {Object} UpperLeg : normalized rotation values
	* @param {Object} LowerLeg : normalized rotation values
	* @param {Side} side : left or right
	*/
	static public RigLeg rigLeg(Vector3 UpperLeg, Vector3 LowerLeg, Side side = Side.Right)
	{
		var invert = side == Side.Right ? 1.0f : -1.0f;

		var rigedUpperLeg = new Vector3()
		{
			x = Mathf.Clamp(UpperLeg.x, 0.0f, 0.5f) * Mathf.PI,
			y = Mathf.Clamp(UpperLeg.y, -0.25f, 0.25f) * Mathf.PI,
			z = Mathf.Clamp(UpperLeg.z, -0.5f, 0.5f) * Mathf.PI + invert * 0.1f,
		};

		var rigedLowerLeg = new Vector3()
		{
			x = LowerLeg.x * Mathf.PI,
			y = LowerLeg.y * Mathf.PI,
			z = LowerLeg.z * Mathf.PI,
		};

		return new RigLeg()
		{
			UpperLeg = rigedUpperLeg,
			LowerLeg = rigedLowerLeg,
		};
	}



	/**
	* Calculate roll, pitch, yaw, centerpoint, and rough dimentions of face plane
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public HeadPlane calcHead(Vector4[] lm)
	{
		// find 3 vectors that form a plane to represent the head
		var plane = createEulerPlane(lm).vector;
		// calculate roll pitch and yaw from vectors
		var rotate = ExtensionVector.rollPitchYaw(plane[0], plane[1], plane[2]);
		// find the center of the face detection box
		var midPoint = Vector3.Lerp(plane[0], plane[1], 0.5f);
		// find the dimensions roughly of the face detection box
		var width = (plane[0] - plane[1]).magnitude;
		var height = (midPoint - plane[2]).magnitude;

		//flip
		rotate.x *= -1;
		rotate.z *= -1;

		return new HeadPlane()
		{
			//defaults to radians for rotation around x,y,z axis
			y = rotate.y,// * Mathf.PI, //left right
			x = rotate.x,// * Mathf.PI, //up down
			z = rotate.z,// * Mathf.PI, //side to side
			width = width,
			height = height,
			//center of face detection square
			position = Vector3.Lerp(midPoint, plane[2], 0.5f),
			//returns euler angles normalized between -1 and 1
			normalized = new Vector3()
			{
				y = rotate.y,
				x = rotate.x,
				z = rotate.z,
			},
			degrees = new Vector3()
			{
				y = rotate.y * 180,
				x = rotate.x * 180,
				z = rotate.z * 180,
			},
		};
	}

	/**
	* Calculate stable plane (triangle) from 4 face landmarks
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public EulerPlane createEulerPlane(Vector4[] lm)
	{
		//create face detection square bounds
		Vector3 p1 = lm[21]; //top left
		Vector3 p2 = lm[251]; //top right
		Vector3 p3 = lm[397]; //bottom right
		Vector3 p4 = lm[172]; //bottom left
		Vector3 p3mid = Vector3.Lerp(p3, p4, 0.5f); // bottom midpoint
		return new EulerPlane()
		{
			vector = new Vector3[] { p1, p2, p3mid },
			points = new Vector3[] { p1, p2, p3, p4 },
		};
	}

	/**
	* Calculate Mouth Shape
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public MouthInfo calcMouth(Vector4[] lm)
	{
		// eye keypoints
		Vector3 eyeInnerCornerL = lm[133];
		Vector3 eyeInnerCornerR = lm[362];
		Vector3 eyeOuterCornerL = lm[130];
		Vector3 eyeOuterCornerR = lm[263];

		// eye keypoint distances
		var eyeInnerDistance = (eyeInnerCornerL - eyeInnerCornerR).magnitude;
		var eyeOuterDistance = (eyeOuterCornerL - eyeOuterCornerR).magnitude;

		// mouth keypoints
		Vector3 upperInnerLip = lm[13];
		Vector3 lowerInnerLip = lm[14];
		Vector3 mouthCornerLeft = lm[61];
		Vector3 mouthCornerRight = lm[291];

		// mouth keypoint distances
		var mouthOpen = (upperInnerLip - lowerInnerLip).magnitude;
		var mouthWidth = (mouthCornerLeft - mouthCornerRight).magnitude;

		// mouth open and mouth shape ratios
		// let ratioXY = mouthWidth / mouthOpen;
		var ratioY = mouthOpen / eyeInnerDistance;
		var ratioX = mouthWidth / eyeOuterDistance;

		// normalize and scale mouth open
		ratioY = ExtensionVector.remap(ratioY, 0.15f, 0.7f);

		// normalize and scale mouth shape
		ratioX = ExtensionVector.remap(ratioX, 0.45f, 0.9f);
		ratioX = (ratioX - 0.3f) * 2.0f;

		// const mouthX = remap(ratioX - 0.4, 0, 0.5);
		var mouthX = ratioX;
		var mouthY = ExtensionVector.remap(mouthOpen / eyeInnerDistance, 0.17f, 0.5f);

		//Depricated: Change sensitivity due to facemesh and holistic have different point outputs.
		// const fixFacemesh = runtime === "tfjs" ? 1.3 : 0;

		// let ratioI = remap(mouthXY, 1.3 + fixFacemesh * 0.8, 2.6 + fixFacemesh) * remap(mouthY, 0, 1);
		var ratioI = ExtensionVector.clamp(ExtensionVector.remap(mouthX, 0, 1) * 2 * ExtensionVector.remap(mouthY, 0.2f, 0.7f), 0.0f, 1.0f);
		var ratioA = mouthY * 0.4f + mouthY * (1.0f - ratioI) * 0.6f;
		var ratioU = mouthY * ExtensionVector.remap(1.0f - ratioI, 0.0f, 0.3f) * 0.1f;
		var ratioE = ExtensionVector.remap(ratioU, 0.2f, 1.0f) * (1.0f - ratioI) * 0.3f;
		var ratioO = (1.0f - ratioI) * ExtensionVector.remap(mouthY, 0.3f, 1.0f) * 0.4f;

		return new MouthInfo()
		{
			x = ratioX,
			y = ratioY,
			shape = new MouthShape()
			{
				A = ratioA,
				E = ratioE,
				I = ratioI,
				O = ratioO,
				U = ratioU,
			},
		};
	}


	/**
	* Calculate eye open ratios and remap to 0-1
	* @param {Array} lm : array of results from tfjs or mediapipe
	* @param {Side} side : designate left or right
	* @param {Number} high : ratio at which eye is considered open
	* @param {Number} low : ratio at which eye is comsidered closed
	*/
	static public EyeOpen getEyeOpen(Vector4[] lm, Side side = Side.Left, float high = 0.85f, float low = 0.55f)
	{
		var eyePoints = Points.eye[(int)side];
		var eyeDistance = eyeLidRatio(
			lm[eyePoints[0]],
			lm[eyePoints[1]],
			lm[eyePoints[2]],
			lm[eyePoints[3]],
			lm[eyePoints[4]],
			lm[eyePoints[5]],
			lm[eyePoints[6]],
			lm[eyePoints[7]]
		);
		// human eye width to height ratio is roughly .3
		var maxRatio = 0.285f;
		// compare ratio against max ratio
		var ratio = ExtensionVector.clamp(eyeDistance / maxRatio, 0.0f, 2.0f);
		// remap eye open and close ratios to increase sensitivity
		var eyeOpenRatio = ExtensionVector.remap(ratio, low, high);
		return new EyeOpen()
		{
			// remapped ratio
			norm = eyeOpenRatio,
			// ummapped ratio
			raw = ratio,
		};
	}

	/**
	* Calculate eyelid distance ratios based on landmarks on the face
	*/
	static public float eyeLidRatio(
		Vector3 eyeOuterCorner,
		Vector3 eyeInnerCorner,
		Vector3 eyeOuterUpperLid,
		Vector3 eyeMidUpperLid,
		Vector3 eyeInnerUpperLid,
		Vector3 eyeOuterLowerLid,
		Vector3 eyeMidLowerLid,
		Vector3 eyeInnerLowerLid
	)
	{
		//use 2D Distances instead of 3D for less jitter
		var eyeWidth = (eyeOuterCorner - eyeInnerCorner).magnitude;
		var eyeOuterLidDistance = (eyeOuterUpperLid - eyeOuterLowerLid).magnitude;
		var eyeMidLidDistance = (eyeMidUpperLid - eyeMidLowerLid).magnitude;
		var eyeInnerLidDistance = (eyeInnerUpperLid - eyeInnerLowerLid).magnitude;
		var eyeLidAvg = (eyeOuterLidDistance + eyeMidLidDistance + eyeInnerLidDistance) / 3;
		var ratio = eyeLidAvg / eyeWidth;

		return ratio;
	}

	/**
	* Calculate pupil position [-1,1]
	* @param {Results} lm : array of results from tfjs or mediapipe
	* @param {Side} side : left or right
	*/
	static public PupilPos pupilPos(Vector4[] lm, Side side = Side.Left)
	{
		Vector2 eyeOuterCorner = lm[Points.eye[(int)side][0]];
		Vector2 eyeInnerCorner = lm[Points.eye[(int)side][1]];
		var eyeWidth = (eyeOuterCorner - eyeInnerCorner).magnitude;
		var midPoint = Vector2.Lerp(eyeOuterCorner, eyeInnerCorner, 0.5f);
		Vector2 pupil = lm[Points.pupil[(int)side][0]];
		var dx = midPoint.x - pupil.x;
		//eye center y is slightly above midpoint
		var dy = midPoint.y - eyeWidth * 0.075f - pupil.y;
		var ratioX = dx / (eyeWidth / 2.0f);
		var ratioY = dy / (eyeWidth / 4.0f);

		ratioX *= 4.0f;
		ratioY *= 4.0f;

		return new PupilPos() { x = ratioX, y = ratioY };
	}

	/**
	* Method to stabilize blink speeds to fix inconsistent eye open/close timing
	* @param {Object} eye : object with left and right eye values
	* @param {Number} headY : head y axis rotation in radians
	* @param {Object} options: Options for blink stabilization
	*/
	static public EyeInfo stabilizeBlink(EyeInfo eye, float headY, bool enableWink = true, float maxRot = 0.5f)
	{
		eye.right = ExtensionVector.clamp(eye.right, 0.0f, 1.0f);
		eye.left = ExtensionVector.clamp(eye.left, 0.0f, 1.0f);
		//difference between each eye
		var blinkDiff = Mathf.Abs(eye.left - eye.right);
		//theshold to which difference is considered a wink
		var blinkThresh = enableWink ? 0.8f : 1.2f;
		//detect when both eyes are closing
		var isClosing = eye.left < 0.3f && eye.right < 0.3f;
		//detect when both eyes are opening
		var isOpen = eye.left > 0.6f && eye.right > 0.6f;

		// sets obstructed eye to the opposite eye value
		if (headY > maxRot)
		{
			return new EyeInfo() { left = eye.right, right = eye.right };
		}
		if (headY < -maxRot)
		{
			return new EyeInfo() { left = eye.left, right = eye.left };
		}

		// returns either a wink or averaged blink values
		return new EyeInfo()
		{
			left =
			blinkDiff >= blinkThresh && !isClosing && !isOpen
				? eye.left
				: eye.right > eye.left
				? Mathf.Lerp(eye.right, eye.left, 0.95f)
				: Mathf.Lerp(eye.right, eye.left, 0.05f),
			right =
			blinkDiff >= blinkThresh && !isClosing && !isOpen
				? eye.right
				: eye.right > eye.left
				? Mathf.Lerp(eye.right, eye.left, 0.95f)
				: Mathf.Lerp(eye.right, eye.left, 0.05f),
		};
	}

	/**
	* Calculate Eyes
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public EyeInfo calcEyes(Vector4[] lm, float high = 0.85f, float low = 0.55f)
	{
		//return early if no iris tracking
		if (lm.Length != 478)
		{
			return new EyeInfo()
			{
				left = 1.0f,
				right = 1.0f,
			};
		}
		//open [0,1]
		var leftEyeLid = getEyeOpen(lm, Side.Left, high, low);
		var rightEyeLid = getEyeOpen(lm, Side.Right, high, low);

		return new EyeInfo()
		{
			left = leftEyeLid.norm,
			right = rightEyeLid.norm,
		};
	}

	/**
	* Calculate pupil location normalized to eye bounds
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public PupilPos calcPupils(Vector4[] lm)
	{
		//pupil x:[-1,1],y:[-1,1]
		if (lm.Length != 478)
		{
			return new PupilPos() { x = 0.0f, y = 0.0f };
		}
		else
		{
			//track pupils using left eye
			var pupilL = pupilPos(lm, Side.Left);
			var pupilR = pupilPos(lm, Side.Right);

			return new PupilPos()
			{
				x = (pupilL.x + pupilR.x) * 0.5f,
				y = (pupilL.y + pupilR.y) * 0.5f,
			};
		}
	}

	/**
	* Calculate brow raise
	* @param {Results} lm : array of results from tfjs or mediapipe
	* @param {Side} side : designate left or right
	*/
	static public float getBrowRaise(Vector4[] lm, Side side = Side.Left)
	{
		var browPoints = Points.brow[(int)side];
		var browDistance = eyeLidRatio(
			lm[browPoints[0]],
			lm[browPoints[1]],
			lm[browPoints[2]],
			lm[browPoints[3]],
			lm[browPoints[4]],
			lm[browPoints[5]],
			lm[browPoints[6]],
			lm[browPoints[7]]
		);

		var maxBrowRatio = 1.15f;
		var browHigh = 0.125f;
		var browLow = 0.07f;
		var browRatio = browDistance / maxBrowRatio - 1.0f;
		var browRaiseRatio = (ExtensionVector.clamp(browRatio, browLow, browHigh) - browLow) / (browHigh - browLow);
		return browRaiseRatio;
	}

	/**
	* Take the average of left and right eyebrow raise values
	* @param {Array} lm : array of results from tfjs or mediapipe
	*/
	static public float calcBrow(Vector4[] lm)
	{
		if (lm.Length != 478)
		{
			return 0.0f;
		}
		else
		{
			var leftBrow = getBrowRaise(lm, Side.Left);
			var rightBrow = getBrowRaise(lm, Side.Right);
			return (leftBrow + rightBrow) / 2.0f;
		}
	}



	/**
     * Calculates finger and wrist as euler rotations
     * @param {Array} lm : array of 3D hand vectors from tfjs or mediapipe
     * @param {Side} side: left or right
     */
	static public Dictionary<string, Vector3> calcHand(Vector4[] lm, Side side = Side.Right)
	{
		Vector3[] palm = new Vector3[] {
			lm[0],
			lm[side == Side.Right ? 17 : 5],
			lm[side == Side.Right ? 5 : 17],
		};

		var handRotation = ExtensionVector.rollPitchYaw(palm[0], palm[1], palm[2]);
		handRotation.y = handRotation.z;
		handRotation.y -= side == Side.Left ? 0.4f : 0.4f;

		var hand = new Dictionary<string, Vector3>();
		hand.Add(side + "Wrist", new Vector3() { x = handRotation.x, y = handRotation.y, z = handRotation.z });
		hand.Add(side + "RingProximal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[0], lm[13], lm[14]) });
		hand.Add(side + "RingIntermediate", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[13], lm[14], lm[15]) });
		hand.Add(side + "RingDistal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[14], lm[15], lm[16]) });
		hand.Add(side + "IndexProximal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[0], lm[5], lm[6]) });
		hand.Add(side + "IndexIntermediate", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[5], lm[6], lm[7]) });
		hand.Add(side + "IndexDistal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[6], lm[7], lm[8]) });
		hand.Add(side + "MiddleProximal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[0], lm[9], lm[10]) });
		hand.Add(side + "MiddleIntermediate", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[9], lm[10], lm[11]) });
		hand.Add(side + "MiddleDistal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[10], lm[11], lm[12]) });
		hand.Add(side + "ThumbProximal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[0], lm[1], lm[2]) });
		hand.Add(side + "ThumbIntermediate", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[1], lm[2], lm[3]) });
		hand.Add(side + "ThumbDistal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[2], lm[3], lm[4]) });
		hand.Add(side + "LittleProximal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[0], lm[17], lm[18]) });
		hand.Add(side + "LittleIntermediate", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[17], lm[18], lm[19]) });
		hand.Add(side + "LittleDistal", new Vector3() { x = 0.0f, y = 0.0f, z = ExtensionVector.angleBetween3DCoords(lm[18], lm[19], lm[20]) });

		hand = rigFingers(hand, side);

		return hand;
	}

	/**
	* Converts normalized rotation values into radians clamped by human limits
	* @param {Object} hand : object of labeled joint with normalized rotation values
	* @param {Side} side : left or right
	*/
	static public Dictionary<string, Vector3> rigFingers(Dictionary<string, Vector3> hand, Side side = Side.Right)
	{
		// Invert modifier based on left vs right side
		var invert = side == Side.Right ? 1.0f : -1.0f;
		string[] digits = { "Ring", "Index", "Little", "Thumb", "Middle" };
		string[] segments = { "Proximal", "Intermediate", "Distal" };

		var wristValue = hand[side + "Wrist"];
		wristValue.x = ExtensionVector.clamp(wristValue.x * 2 * invert, -0.3f, 0.3f); // twist
		wristValue.y = ExtensionVector.clamp(
			wristValue.y * 2.3f,
			side == Side.Right ? -1.2f : -0.6f,
			side == Side.Right ? 0.6f : 1.6f
		);
		wristValue.z = wristValue.z * -2.3f * invert; //left right
		hand[side + "Wrist"] = wristValue;

		foreach (var e in digits)
		{
			foreach (var j in segments)
			{
				var trackedFinger = hand[side + e + j];

				if (e == "Thumb")
				{
					//dampen thumb rotation depending on segment
					var dampener = new Vector3()
					{
						x = j == "Proximal" ? 2.2f : j == "Intermediate" ? 0.0f : 0.0f,
						y = j == "Proximal" ? 2.2f : j == "Intermediate" ? 0.7f : 1.0f,
						z = j == "Proximal" ? 0.5f : j == "Intermediate" ? 0.5f : 0.5f,
					};
					var startPos = new Vector3()
					{
						x = j == "Proximal" ? 1.2f : j == "Distal" ? -0.2f : -0.2f,
						y = j == "Proximal" ? 1.1f * invert : j == "Distal" ? 0.1f * invert : 0.1f * invert,
						z = j == "Proximal" ? 0.2f * invert : j == "Distal" ? 0.2f * invert : 0.2f * invert,
					};
					var newThumb = Vector3.zero;
					if (j == "Proximal")
					{
						newThumb.z = ExtensionVector.clamp(
							startPos.z + trackedFinger.z * -Mathf.PI * dampener.z * invert,
							side == Side.Right ? -0.6f : -0.3f,
							side == Side.Right ? 0.3f : 0.6f
						);
						newThumb.x = ExtensionVector.clamp(startPos.x + trackedFinger.z * -Mathf.PI * dampener.x, -0.6f, 0.3f);
						newThumb.y = ExtensionVector.clamp(
							startPos.y + trackedFinger.z * -Mathf.PI * dampener.y * invert,
							side == Side.Right ? -1.0f : -0.3f,
							side == Side.Right ? 0.3f : 1.0f
						);
					}
					else
					{
						newThumb.z = ExtensionVector.clamp(startPos.z + trackedFinger.z * -Mathf.PI * dampener.z * invert, -2.0f, 2.0f);
						newThumb.x = ExtensionVector.clamp(startPos.x + trackedFinger.z * -Mathf.PI * dampener.x, -2.0f, 2.0f);
						newThumb.y = ExtensionVector.clamp(startPos.y + trackedFinger.z * -Mathf.PI * dampener.y * invert, -2.0f, 2.0f);
					}
					trackedFinger.x = newThumb.x;
					trackedFinger.y = newThumb.y;
					trackedFinger.z = newThumb.z;
				}
				else
				{
					//will document human limits later
					trackedFinger.z = ExtensionVector.clamp(
						trackedFinger.z * -Mathf.PI * invert,
						side == Side.Right ? -Mathf.PI : 0,
						side == Side.Right ? 0 : Mathf.PI
					);
				}
			}
		}

		return hand;
	}

	/**
	* Calculates Hip rotation and world position
	* @param {Array} lm3d : array of 3D pose vectors from tfjs or mediapipe
	* @param {Array} lm2d : array of 2D pose vectors from tfjs or mediapipe
	*/
	static public HipInfo calcHips(Vector4[] lm)
	{
		//Find 2D normalized Hip and Shoulder Joint Positions/Distances
		Vector3 hipLeft2d = lm[23];
		Vector3 hipRight2d = lm[24];
		Vector3 shoulderLeft2d = lm[11];
		Vector3 shoulderRight2d = lm[12];
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

		hips.rotation = ExtensionVector.rollPitchYaw(lm[23], lm[24]);
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

		var spine = ExtensionVector.rollPitchYaw(lm[11], lm[12]);
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

	/**
	* Converts normalized rotations to radians and estimates world position of hips
	* @param {Object} hips : hip position and rotation values
	* @param {Object} spine : spine position and rotation values
	*/
	static public HipInfo rigHips(IHips hips, Vector3 spine)
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