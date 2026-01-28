using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;


public class ValueSubscriber : MonoBehaviour
{
    ROSConnection ros;
    private float receivedValue;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32Msg>("my_value", Callback);
    }

    void Callback(Float32Msg msg)
    {
        receivedValue = msg.data;
    }

    public float GetReceivedValue()
    {
        return receivedValue;
    }
}
