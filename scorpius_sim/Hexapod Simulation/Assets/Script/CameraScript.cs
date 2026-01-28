using UnityEngine;

public class CameraScript : MonoBehaviour
{
    private Camera cam;
    [SerializeField] private float minX = -10f;
    [SerializeField] private float minY = -10f;
    [SerializeField] private float maxX = 10f;
    [SerializeField] private float maxY = 10f;
    [SerializeField] private float moveSpeed = 1f;
    [SerializeField] private float zoomSpeed = 1f;
    [SerializeField] private float minFOV = 1f;
    [SerializeField] private float maxFOV = 1f;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        cam = GetComponent<Camera>();
    }

    // Update is called once per frame
    void Update()
    {
        // Move camera position
        if (Input.GetKey(KeyCode.W))
        {
            transform.position += moveSpeed * Time.deltaTime * Vector3.forward;
        }
        if (Input.GetKey(KeyCode.D))
        {
            transform.position += moveSpeed * Time.deltaTime * Vector3.right;
        }
        if (Input.GetKey(KeyCode.S))
        {
            transform.position += moveSpeed * Time.deltaTime * Vector3.back;
        }
        if (Input.GetKey(KeyCode.A))
        {
            transform.position += moveSpeed * Time.deltaTime * Vector3.left;
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
}
