using System;
using MyExtensions;
using UnityEditor;
using UnityEngine;


public class GeometryTrigger : MonoBehaviour
{
	private enum TriggerMode
	{
		None,
		CylinderSector,
		Spherical,
		SphericalSector
	}

	[SerializeField] private TriggerMode mode = TriggerMode.None;
	[SerializeField] private Transform debugTarget;
	[Space]
	[SerializeField] private float minRadius;
	[SerializeField] private float maxRadius;
	[SerializeField] private float fovAngle;
	[SerializeField] private float height;
	[SerializeField] private float distance;


	private void OnDrawGizmos()
	{
		bool isInside = debugTarget != true || IsPointInsideTrigger(debugTarget.position);
		Gizmos.color  = isInside ? Color.green : Color.red;
		Handles.color = isInside ? Color.green : Color.red;

		Gizmos.matrix = Handles.matrix = transform.localToWorldMatrix;

		switch (mode)
		{
		case TriggerMode.CylinderSector:
			DrawCylinderSector();
			break;

		case TriggerMode.Spherical:
			DrawSphericalTrigger();
			break;

		case TriggerMode.SphericalSector:
			DrawSphericalSector();
			break;
		}
	}

	public bool IsPointInsideTrigger(Vector3 worldPoint)
	{
		switch (mode)
		{
		case TriggerMode.CylinderSector: return IsInsideCylinderSector(worldPoint);
		case TriggerMode.Spherical: return IsInsideSpherical(worldPoint);
		case TriggerMode.SphericalSector: return isInsideSphericalSector(worldPoint);

		default: return false;
		}
	}

	private void DrawSphericalSector()
	{
		float sideA = distance;
		float hypotenuse = maxRadius;
		float sideB = Mathf.Sqrt(hypotenuse * hypotenuse - sideA * sideA);

		Handles.DrawWireDisc(Vector3.forward * distance, Vector3.forward, sideB);
		Gizmos.DrawWireSphere(Vector3.zero, maxRadius);

		const int SEGMENTS = 16;
		for (int i = 0; i < SEGMENTS; i++)
		{
			float angle = (float)i / SEGMENTS * 360 * Mathf.Deg2Rad;

			Vector3 pointOnWire = new(Mathf.Cos(angle) * sideB, Mathf.Sin(angle) * sideB, distance);
			Gizmos.DrawLine(Vector3.zero, pointOnWire);
		}
	}

	private void DrawSphericalTrigger()
	{
		Handles.DrawWireDisc(Vector3.zero, Vector3.up, minRadius);
		Handles.DrawWireDisc(Vector3.zero, Vector3.up, maxRadius);

		Handles.DrawWireDisc(Vector3.zero.SetY(height), Vector3.up, minRadius);
		Handles.DrawWireDisc(Vector3.zero.SetY(height), Vector3.up, maxRadius);
	}

	private void DrawCylinderSector()
	{
		float offsetAngle = 90;
		Vector3 startLocalPos = AngleToDirection((offsetAngle + fovAngle / 2) * Mathf.Deg2Rad);
		Vector3 endLocalPos = AngleToDirection((offsetAngle - fovAngle / 2) * Mathf.Deg2Rad);

		Handles.DrawWireArc(Vector3.zero, Vector3.up, startLocalPos, fovAngle, minRadius);
		Handles.DrawWireArc(Vector3.zero, Vector3.up, startLocalPos, fovAngle, maxRadius);

		Vector3 leftMinPos = startLocalPos * minRadius;
		Vector3 leftMaxPos = startLocalPos * maxRadius;
		Gizmos.DrawLine(leftMinPos, leftMaxPos);

		Vector3 rightMinPos = endLocalPos * minRadius;
		Vector3 rightMaxPos = endLocalPos * maxRadius;
		Gizmos.DrawLine(rightMinPos, rightMaxPos);

		Vector3 heightOffset = Vector3.up * height;

		Handles.DrawWireArc(heightOffset, Vector3.up, startLocalPos, fovAngle, minRadius);
		Handles.DrawWireArc(heightOffset, Vector3.up, startLocalPos, fovAngle, maxRadius);

		Vector3 leftMinTopPos = startLocalPos * minRadius + heightOffset;
		Vector3 leftMaxTopPos = startLocalPos * maxRadius + heightOffset;
		Gizmos.DrawLine(leftMinTopPos, leftMaxTopPos);

		Vector3 rightMinTopPos = endLocalPos * minRadius + heightOffset;
		Vector3 rightMaxTopPos = endLocalPos * maxRadius + heightOffset;
		Gizmos.DrawLine(rightMinTopPos, rightMaxTopPos);

		Gizmos.DrawLine(leftMinPos, leftMinTopPos);
		Gizmos.DrawLine(leftMaxPos, leftMaxTopPos);

		Gizmos.DrawLine(rightMinPos, rightMinTopPos);
		Gizmos.DrawLine(rightMaxPos, rightMaxTopPos);
	}

	private Vector3 AngleToDirection(float angle)
		=> new(Mathf.Cos(angle), 0, Mathf.Sin(angle));

	private bool IsInsideCylinderSector(Vector3 worldPoint)
	{
		Vector3 localPos = transform.InverseTransformPoint(worldPoint);

		if (localPos.y < 0 || localPos.y > height)
			return false;

		Vector3 localPosWithoutHeight = localPos.SetY(0.0f);
		float distance = Vector3.Distance(Vector3.zero, localPosWithoutHeight);
		if (distance < minRadius || distance > maxRadius)
			return false;

		float angleToPos = Vector3.SignedAngle(Vector3.forward, localPos, Vector3.up);
		if (angleToPos < -(fovAngle / 2) || angleToPos > fovAngle / 2)
			return false;

		return true;
	}

	private bool IsInsideSpherical(Vector3 worldPoint)
	{
		Vector3 localPos = transform.InverseTransformPoint(worldPoint);

		if (localPos.y < 0 || localPos.y > height)
			return false;

		Vector3 localPosWithoutHeight = localPos.SetY(0.0f);
		float distance = Vector3.Distance(Vector3.zero, localPosWithoutHeight);
		if (distance < minRadius || distance > maxRadius)
			return false;

		return true;
	}

	private bool isInsideSphericalSector(Vector3 worldPoint)
	{
		Vector3 localPos = transform.InverseTransformPoint(worldPoint);

		float distance = Vector3.Distance(Vector3.zero, localPos);
		if (distance < minRadius || distance > maxRadius)
			return false;

		float angleToPos = Vector3.SignedAngle(Vector3.forward, localPos, Vector3.forward);
		float side = Mathf.Sqrt(maxRadius * maxRadius - this.distance * this.distance);
		Vector3 posOnEdgeOfMaxRadius = new(side, 0, this.distance);
		float angleToEdgeOfMaxRadius = Vector3.SignedAngle(Vector3.forward, posOnEdgeOfMaxRadius, Vector3.forward);

		if (angleToPos > angleToEdgeOfMaxRadius)
			return false;

		return true;
	}
}