using UnityEngine;
using UnityEngine.Rendering;

public class ArduinoScript : MonoBehaviour
{
    // Vertical servomotors
    public ServomotorScript servoVertA;
    public ServomotorScript servoVertB;
    public ServomotorScript servoVertC;
    public ServomotorScript servoVertD;
    public ServomotorScript servoVertE;
    public ServomotorScript servoVertF;

    public bool EnableVerticalServos = false;
    public bool up = true;
    public float upAngle = 45;
    public float downAngle = -45f;

    // Horizontal servomotors
    public ServomotorScript servoHorizA;
    public ServomotorScript servoHorizB;
    public ServomotorScript servoHorizC;
    public ServomotorScript servoHorizD;
    public ServomotorScript servoHorizE;
    public ServomotorScript servoHorizF;

    public bool EnableHorizontalServos = false;
    public bool forward = true;
    public float forwardAngle = 45f;
    public float backwardAngle = -45f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Vertical servomotors
        if (EnableVerticalServos)
        {
            if (up)
            {
                float angle = Mathf.Clamp(-upAngle + 90f, -45f, 65f);
                servoVertA.targetAngle = angle;
                servoVertB.targetAngle = angle;
                servoVertC.targetAngle = angle;
                servoVertD.targetAngle = angle;
                servoVertE.targetAngle = angle;
                servoVertF.targetAngle = angle;
            }
            else
            {
                float angle = Mathf.Clamp(-downAngle + 90f, -45f, 65f);
                servoVertA.targetAngle = angle;
                servoVertB.targetAngle = angle;
                servoVertC.targetAngle = angle;
                servoVertD.targetAngle = angle;
                servoVertE.targetAngle = angle;
                servoVertF.targetAngle = angle;
            }
        }
        else
        {
            servoVertA.targetAngle = 0f;
            servoVertB.targetAngle = 0f;
            servoVertC.targetAngle = 0f;
            servoVertD.targetAngle = 0f;
            servoVertE.targetAngle = 0f;
            servoVertF.targetAngle = 0f;
        }

        // Horizontal servomotors
        if (EnableHorizontalServos)
        {
            if (forward)
            {
                servoHorizA.targetAngle = forwardAngle;
                servoHorizB.targetAngle = forwardAngle;
                servoHorizC.targetAngle = forwardAngle;
                servoHorizD.targetAngle = forwardAngle;
                servoHorizE.targetAngle = forwardAngle;
                servoHorizF.targetAngle = forwardAngle;
            }
            else
            {
                servoHorizA.targetAngle = backwardAngle;
                servoHorizB.targetAngle = backwardAngle;
                servoHorizC.targetAngle = backwardAngle;
                servoHorizD.targetAngle = backwardAngle;
                servoHorizE.targetAngle = backwardAngle;
                servoHorizF.targetAngle = backwardAngle;
            }
        }
        else
        {
            servoHorizA.targetAngle = 0f;
            servoHorizB.targetAngle = 0f;
            servoHorizC.targetAngle = 0f;
            servoHorizD.targetAngle = 0f;
            servoHorizE.targetAngle = 0f;
            servoHorizF.targetAngle = 0f;
        }
    }
}
