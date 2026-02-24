using UnityEngine;

public class CameraScript : MonoBehaviour
{
    private Camera cam;
    [SerializeField] private bool sideView = false;
    [SerializeField] private Transform sideViewTransform;
    [SerializeField] private Transform moveViewTransform;
    [SerializeField] private float minX = -10f;
    [SerializeField] private float minY = -10f;
    [SerializeField] private float maxX = 10f;
    [SerializeField] private float maxY = 10f;
    [SerializeField] private float moveSpeed = 1f;
    [SerializeField] private float moveSpeedMultiplier = 2f;
    [SerializeField] private float zoomSpeed = 1f;
    [SerializeField] private float minFOV = 1f;
    [SerializeField] private float maxFOV = 1f;
    [SerializeField] private float FOV;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        cam = GetComponent<Camera>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            sideView = !sideView;
            if (sideView)
            {
                moveViewTransform.position = transform.position;
                moveViewTransform.rotation = transform.rotation;
                transform.position = sideViewTransform.position;
                transform.rotation = sideViewTransform.rotation;
                FOV = cam.fieldOfView;
                cam.fieldOfView = 65f;
            }
            else
            {
                transform.position = moveViewTransform.position;
                transform.rotation = moveViewTransform.rotation;
                cam.fieldOfView = FOV;
            }
        }

        if (!sideView)
            {
                // Move camera position
                float speed = moveSpeed;
                if (Input.GetKey(KeyCode.LeftShift))
                {
                    speed *= moveSpeedMultiplier;
                }
                if (Input.GetKey(KeyCode.W))
                {
                    transform.position += speed * Time.deltaTime * Vector3.forward;
                }
                if (Input.GetKey(KeyCode.D))
                {
                    transform.position += speed * Time.deltaTime * Vector3.right;
                }
                if (Input.GetKey(KeyCode.S))
                {
                    transform.position += speed * Time.deltaTime * Vector3.back;
                }
                if (Input.GetKey(KeyCode.A))
                {
                    transform.position += speed * Time.deltaTime * Vector3.left;
                }


                // Clamp position
                if (transform.position.x < minX)
                {
                    transform.position = new Vector3(minX, transform.position.y, transform.position.z);
                }
                if (transform.position.x > maxX)
                {
                    transform.position = new Vector3(maxX, transform.position.y, transform.position.z);
                }
                if (transform.position.z < minY)
                {
                    transform.position = new Vector3(transform.position.x, transform.position.y, minY);
                }
                if (transform.position.z > maxY)
                {
                    transform.position = new Vector3(transform.position.x, transform.position.y, maxY);
                }

                // Zoom camera
                float deltaZoom = -Input.GetAxis("Mouse ScrollWheel");
                cam.fieldOfView += deltaZoom * zoomSpeed * 10000 * Time.deltaTime;

                // Clamp zoom
                cam.fieldOfView = Mathf.Clamp(cam.fieldOfView, minFOV, maxFOV);
            }
            else
            {
                // Side view
                transform.position = new Vector3(3.6f, 3.7f, -2.1f);
                transform.rotation = Quaternion.Euler(45f, -60f, 0f);
                cam.fieldOfView = 65f;
            }
    }
}
