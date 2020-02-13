using System.Collections;
using UnityEngine;

public class CameraController : MonoBehaviour {
    public Transform target;

    void Update () {
        if (Input.GetMouseButtonDown (0)) {
            RaycastHit hit;
            Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
            if (Physics.Raycast (ray, out hit, 1000.0f)) {
                Vector3 newPos = new Vector3 (hit.point.x, hit.point.y, target.position.z);
                target.position = newPos;
            }
        }
    }
}