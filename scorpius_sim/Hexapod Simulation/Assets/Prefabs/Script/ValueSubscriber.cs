using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;


public class ValueSubscriber : MonoBehaviour
{
    ROSConnection ros;
    private float[] servoAngles = new float[12];

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32Msg>("ServoAngles", Callback);
    }

    void Callback(Float32Msg msg)
    {
        //servoAngles[0] = msg.vert_a
        //servoAngles[1] = msg.vert_b
        //servoAngles[2] = msg.vert_c
        //servoAngles[3] = msg.vert_d
        //servoAngles[4] = msg.vert_e
        //servoAngles[5] = msg.vert_f
        //servoAngles[6] = msg.horiz_a
        //servoAngles[7] = msg.horiz_b
        //servoAngles[8] = msg.horiz_c
        //servoAngles[9] = msg.horiz_d
        //servoAngles[10] = msg.horiz_e
        //servoAngles[11] = msg.horiz_f

    }

    public float[] GetReceivedValue()
    {
        return servoAngles;
    }
}
