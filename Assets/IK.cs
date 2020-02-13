using UnityEngine;
using UnityEngine.UI;

public class IK : MonoBehaviour {
    public Toggle toggleF;
    public Toggle toggleJ;
    public int IKChainLength = 4;
    public Transform Target;
    [Range (0.1f, 2f)]
    public float stride = 1f;
    public float stoppingDelta = 0.001f;

    protected bool isFabrik;
    protected Transform root;
    protected Transform[] jointArray;
    protected float[] jointLengthArray;
    protected float TotalRigLength;

    void Awake () {
        isFabrik = false;
        jointArray = new Transform[IKChainLength + 1];
        jointLengthArray = new float[IKChainLength];

        //find root
        root = transform;
        for (var i = 0; i <= IKChainLength; i++) {
            root = root.parent;
        }
        var current = transform;
        TotalRigLength = 0;
        for (var i = jointArray.Length - 1; i >= 0; i--) {
            jointArray[i] = current;

            if (i != jointArray.Length - 1) {
                jointLengthArray[i] = (GetPos (jointArray[i + 1]) - GetPos (current)).magnitude;
                TotalRigLength += jointLengthArray[i];
            }
            current = current.parent;
        }
    }

    public void OnToggleFValueChanged (bool selected) {
        if (selected) {
            toggleJ.isOn = false;
            isFabrik = true;
        } else {
            toggleJ.isOn = true;
            isFabrik = false;
        }
    }

    public void OnToggleJValueChanged (bool selected) {
        if (selected) {
            toggleF.isOn = false;
            isFabrik = false;
        } else {
            toggleF.isOn = true;
            isFabrik = true;
        }
        // Clicking ToggleF also triggers this function, thus ResetJoints() should be called once
        ResetJoints ();
    }

    void ResetJoints () {
        GameObject[] bones = GameObject.FindGameObjectsWithTag ("Bone");
        if (isFabrik) {
            // Do not render bones in Fabrik mode
            for (int i = 0; i < bones.Length; i++) {
                bones[i].GetComponent<Renderer> ().enabled = false;
            }
            // Reset joint rotations
            jointArray[0].localRotation = Quaternion.Euler (0, 0, 5);
            for (int i = 1; i < jointArray.Length; i++) {
                jointArray[i].localRotation = Quaternion.identity;
            }
        } else {
            // Render bones
            for (int i = 0; i < bones.Length; i++) {
                bones[i].GetComponent<Renderer> ().enabled = true;
            }
            // Reset joint positions
            jointArray[0].localPosition = new Vector3 (0, 0, 0);
            jointArray[1].localPosition = new Vector3 (13.5f, 0, 0);
            jointArray[2].localPosition = new Vector3 (11, 0, 0);
            jointArray[3].localPosition = new Vector3 (3.5f, 0, 0);
            jointArray[4].localPosition = new Vector3 (4, 0, 0);
        }
    }

    void LateUpdate () {
        var targetPos = GetPos (Target);
        // Get jointArray
        var current = transform;
        for (var i = jointArray.Length - 1; i >= 0; i--) {
            jointArray[i] = current;
            current = current.parent;
        }

        var rigDistance = GetPos (jointArray[IKChainLength]) - targetPos;
        // If it does reach the target
        if ((rigDistance).magnitude < stoppingDelta) {
            // Stop the rig if close enough
            return;
        }

        var targetDistance = targetPos - GetPos (jointArray[0]);
        //If it is not possible to reach the target
        if ((targetDistance).magnitude >= TotalRigLength) {
            if (isFabrik) {
                // Set joint positions in a line
                for (int i = 0; i < jointArray.Length - 1; i++) {
                    float ratio = jointLengthArray[i] / (targetPos - GetPos (jointArray[i])).magnitude;
                    SetPos (jointArray[i + 1], (1 - ratio) * GetPos (jointArray[i]) + ratio * targetPos);
                }
            } else {
                // Rotate joints in a line
                for (int i = 0; i < jointArray.Length; i++) {
                    SetRot (jointArray[i], Quaternion.FromToRotation (new Vector3 (1, 0, 0), targetPos));
                }
            }

        } else {
            if (isFabrik) {
                Vector3 startPos = GetPos (jointArray[0]);
                // Fabrik Forward Reaching
                SetPos (jointArray[IKChainLength], targetPos);
                for (int i = jointArray.Length - 2; i >= 0; i--) {
                    float ratio = jointLengthArray[i] / (GetPos (jointArray[i + 1]) - GetPos (jointArray[i])).magnitude;
                    SetPos (jointArray[i], (1 - ratio) * GetPos (jointArray[i + 1]) + ratio * GetPos (jointArray[i]));
                }
                // Fabrik Backward Reaching
                SetPos (jointArray[0], startPos);
                for (int i = 0; i < jointArray.Length - 1; i++) {
                    float ratio = jointLengthArray[i] / (GetPos (jointArray[i + 1]) - GetPos (jointArray[i])).magnitude;
                    SetPos (jointArray[i + 1], (1 - ratio) * GetPos (jointArray[i]) + ratio * GetPos (jointArray[i + 1]));
                }
            } else {
                // Jacobian Pseudo Inverse IK
                double[] deltaOrientation = new double[IKChainLength];
                deltaOrientation = GetDeltaOrientation (targetPos);
                for (int i = 0; i < deltaOrientation.Length; i++) {
                    SetRot (jointArray[i], GetRot (jointArray[i]) * Quaternion.Euler (0, 0, (float) deltaOrientation[i] * stride));
                }
            }
        }
    }

    private double[] GetDeltaOrientation (Vector3 targetPos) {
        double[][] jpiMat = GetJacobianPseudoInverse (targetPos);
        Vector3 deltaPos = targetPos - GetPos (jointArray[IKChainLength]);
        double[] d = { deltaPos.x, deltaPos.y };
        return MatrixUtil.MatrixVectorProduct (jpiMat, d);
    }

    private double[][] GetJacobianPseudoInverse (Vector3 targetPos) {
        double[][] jMat = MatrixUtil.MatrixCreate (2, IKChainLength);
        double[][] jtMat = MatrixUtil.MatrixCreate (IKChainLength, 2);
        double[][] jpiMat = MatrixUtil.MatrixCreate (IKChainLength, 2);
        for (int i = 0; i < jointArray.Length - 1; i++) {
            Vector3 cross = Vector3.Cross (Vector3.forward, GetPos (jointArray[IKChainLength]) - GetPos (jointArray[i]));
            jtMat[i][0] = cross.x;
            jtMat[i][1] = cross.y;
        }
        jMat = MatrixUtil.MatrixTranspose (jtMat);
        jpiMat = MatrixUtil.MatrixProduct (jtMat, MatrixUtil.MatrixInverse (MatrixUtil.MatrixProduct (jMat, jtMat)));
        return jpiMat;
    }

    private Quaternion GetRot (Transform current) {
        if (root == null) {
            return current.rotation;
        } else {
            // return Quaternion.Inverse (current.rotation) * root.rotation;
            return Quaternion.Inverse (root.rotation) * current.rotation;
        }
    }

    private void SetRot (Transform current, Quaternion rotation) {
        if (root == null) {
            current.rotation = rotation;
        } else {
            current.rotation = root.rotation * rotation;
        }
    }

    private Vector3 GetPos (Transform current) {
        if (root == null) {
            return current.position;
        } else {
            return Quaternion.Inverse (root.rotation) * (current.position - root.position);
        }
    }

    private void SetPos (Transform current, Vector3 position) {
        if (root == null) {
            current.position = position;
        } else {
            current.position = root.rotation * position + root.position;
        }
    }
}