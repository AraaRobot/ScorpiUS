using Unity.VisualScripting;
using UnityEngine;

/// <summary>
/// This script simulates a servomotor by applying torque to a Rigidbody to reach a target angle. Most be a child of the rigidbody that is rotating.
/// The Z axis points in the diraction of the servomotor while the X axis is the rotation axis. The initial forward vector is used as the zero angle reference.
/// </summary>s
public class ServomotorScript : MonoBehaviour
{
    [SerializeField] private Rigidbody ControlledRigidBody;
    [SerializeField] private Rigidbody ConnectedRigidBody;
    [SerializeField] private float slowAngle = 10f;
    public float targetAngle = 0f;
    public float torque = 0f;
    private Vector3 rotationAxis;
    private Vector3 initialVector;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // Assert there is a Rigidbody component
        if (ControlledRigidBody == null || ConnectedRigidBody == null) Debug.LogError("Rigidbody component not assigned in ServomotorScript on " + gameObject.name);
    }


    // FixedUpdate is called every fixed framerate frame
    void FixedUpdate()
    {
        // Update the targetAngle
        targetAngle = Mathf.Clamp(targetAngle, -180f, 180f);

        // Update the rotation axis and initial vector
        rotationAxis = transform.right;
        initialVector = Vector3.ProjectOnPlane(Vector3.Project(transform.forward, ConnectedRigidBody.transform.right), rotationAxis).normalized;

        // Calculate the target rotation based on the initial rotation and the target angle
        Vector3 targetVector = Quaternion.AngleAxis(targetAngle, rotationAxis) * initialVector;
        //Debug.Log(Vector3.Angle(initialVector, targetVector));

        // Add torque to reach the target rotation
        if (Vector3.Angle(transform.forward, targetVector) < slowAngle) // snap to target angle if close enough
        {
            ControlledRigidBody.AddTorque(rotationAxis * torque * Time.fixedDeltaTime * Vector3.SignedAngle(transform.forward, targetVector, rotationAxis) / slowAngle); ;
        }
        else // apply torque to rotate towards target angle
        {
            if (Vector3.SignedAngle(transform.forward, targetVector, rotationAxis) < 0)
            {
                ControlledRigidBody.AddTorque(rotationAxis * -torque * Time.fixedDeltaTime);
            }
            else
            {
                ControlledRigidBody.AddTorque(rotationAxis * torque * Time.fixedDeltaTime);
            }
        }

    }
}
