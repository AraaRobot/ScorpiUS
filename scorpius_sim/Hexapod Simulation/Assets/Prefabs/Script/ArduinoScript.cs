using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.UIElements;

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
    public float neutralAngle = 0f;
    private int position = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Position inputs
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            position = 1;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            position = 2;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            position = 3;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            position = 4;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            position = 0;
        }

        // Position
        float ua = Mathf.Clamp(-upAngle + 90f, -45f, 65f);
        float da = Mathf.Clamp(-downAngle + 90f, -45f, 65f);
        float na = Mathf.Clamp(-neutralAngle + 90f, -45f, 65f);
        switch (position)
        {
            case 1:
                servoHorizA.targetAngle = forwardAngle;
                servoHorizC.targetAngle = forwardAngle;
                servoHorizE.targetAngle = forwardAngle;
                servoHorizB.targetAngle = backwardAngle;
                servoHorizD.targetAngle = backwardAngle;
                servoHorizF.targetAngle = backwardAngle;
                servoVertA.targetAngle = da;
                servoVertB.targetAngle = da;
                servoVertC.targetAngle = da;
                servoVertD.targetAngle = da;
                servoVertE.targetAngle = da;
                servoVertF.targetAngle = da;
                break;
            case 2:
                servoHorizA.targetAngle = neutralAngle;
                servoHorizC.targetAngle = neutralAngle;
                servoHorizE.targetAngle = neutralAngle;
                servoHorizB.targetAngle = neutralAngle;
                servoHorizD.targetAngle = neutralAngle;
                servoHorizF.targetAngle = neutralAngle;
                servoVertA.targetAngle = ua;
                servoVertB.targetAngle = da;
                servoVertC.targetAngle = ua;
                servoVertD.targetAngle = da;
                servoVertE.targetAngle = ua;
                servoVertF.targetAngle = da;
                break;
            case 3:
                servoHorizA.targetAngle = backwardAngle;
                servoHorizC.targetAngle = backwardAngle;
                servoHorizE.targetAngle = backwardAngle;
                servoHorizB.targetAngle = forwardAngle;
                servoHorizD.targetAngle = forwardAngle;
                servoHorizF.targetAngle = forwardAngle;
                servoVertA.targetAngle = da;
                servoVertB.targetAngle = da;
                servoVertC.targetAngle = da;
                servoVertD.targetAngle = da;
                servoVertE.targetAngle = da;
                servoVertF.targetAngle = da;
                break;
            case 4:
                servoHorizA.targetAngle = neutralAngle;
                servoHorizC.targetAngle = neutralAngle;
                servoHorizE.targetAngle = neutralAngle;
                servoHorizB.targetAngle = neutralAngle;
                servoHorizD.targetAngle = neutralAngle;
                servoHorizF.targetAngle = neutralAngle;
                servoVertA.targetAngle = da;
                servoVertB.targetAngle = ua;
                servoVertC.targetAngle = da;
                servoVertD.targetAngle = ua;
                servoVertE.targetAngle = da;
                servoVertF.targetAngle = ua;
                break;
            case 0:
                servoHorizA.targetAngle = neutralAngle;
                servoHorizC.targetAngle = neutralAngle;
                servoHorizE.targetAngle = neutralAngle;
                servoHorizB.targetAngle = neutralAngle;
                servoHorizD.targetAngle = neutralAngle;
                servoHorizF.targetAngle = neutralAngle;
                servoVertA.targetAngle = neutralAngle;
                servoVertB.targetAngle = neutralAngle;
                servoVertC.targetAngle = neutralAngle;
                servoVertD.targetAngle = neutralAngle;
                servoVertE.targetAngle = neutralAngle;
                servoVertF.targetAngle = neutralAngle;
                break;
            default:
                break;
        }

        // Vertical servomotors
        //if (EnableVerticalServos)
        //{
        //    if (up)
        //    {
        //        float angle = Mathf.Clamp(-upAngle + 90f, -45f, 65f);
        //        servoVertA.targetAngle = angle;
        //        servoVertB.targetAngle = angle;
        //        servoVertC.targetAngle = angle;
        //        servoVertD.targetAngle = angle;
        //        servoVertE.targetAngle = angle;
        //        servoVertF.targetAngle = angle;
        //    }
        //    else
        //    {
        //        float angle = Mathf.Clamp(-downAngle + 90f, -45f, 65f);
        //        servoVertA.targetAngle = angle;
        //        servoVertB.targetAngle = angle;
        //        servoVertC.targetAngle = angle;
        //        servoVertD.targetAngle = angle;
        //        servoVertE.targetAngle = angle;
        //        servoVertF.targetAngle = angle;
        //    }
        //}
        //else
        //{
        //    servoVertA.targetAngle = 0f;
        //    servoVertB.targetAngle = 0f;
        //    servoVertC.targetAngle = 0f;
        //    servoVertD.targetAngle = 0f;
        //    servoVertE.targetAngle = 0f;
        //    servoVertF.targetAngle = 0f;
        //}

        //// Horizontal servomotors
        //if (EnableHorizontalServos)
        //{
        //    if (forward)
        //    {
        //        servoHorizA.targetAngle = forwardAngle;
        //        servoHorizB.targetAngle = forwardAngle;
        //        servoHorizC.targetAngle = forwardAngle;
        //        servoHorizD.targetAngle = forwardAngle;
        //        servoHorizE.targetAngle = forwardAngle;
        //        servoHorizF.targetAngle = forwardAngle;
        //    }
        //    else
        //    {
        //        servoHorizA.targetAngle = backwardAngle;
        //        servoHorizB.targetAngle = backwardAngle;
        //        servoHorizC.targetAngle = backwardAngle;
        //        servoHorizD.targetAngle = backwardAngle;
        //        servoHorizE.targetAngle = backwardAngle;
        //        servoHorizF.targetAngle = backwardAngle;
        //    }
        //}
        //else
        //{
        //    servoHorizA.targetAngle = 0f;
        //    servoHorizB.targetAngle = 0f;
        //    servoHorizC.targetAngle = 0f;
        //    servoHorizD.targetAngle = 0f;
        //    servoHorizE.targetAngle = 0f;
        //    servoHorizF.targetAngle = 0f;
        //}
    }
}
