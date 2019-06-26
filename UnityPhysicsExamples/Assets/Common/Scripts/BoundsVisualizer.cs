using UnityEngine;
using Unity.Bounds;
using Unity.Mathematics;
using Unity.Collections;

public class BoundsVisualizer : MonoBehaviour
{
    public enum HullType
    {
        Octahedral,
        AABB,
        HexPrism,
    };
    public HullType boundsType;

    public bool m_ShowWireframe = true;

    void OnDrawGizmos()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null)
            return;

        UnityEngine.Mesh sharedMesh = meshFilter.sharedMesh;
        if (sharedMesh == null)
            return;

        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        if (meshRenderer == null)
            return;

        Vector3[] sharedMeshVertices = sharedMesh.vertices;
        if (sharedMeshVertices.Length == 0)
            return;

        if (m_ShowWireframe)
        {
            Color32 red = new Color32(255, 0, 0, 255);
            Color32 blue = new Color32(0, 0, 255, 255);
            Color32 yellow = new Color32(255, 255, 0, 255);
            Gizmos.color = red;
            //Unity.Bounds.Mesh mesh = new Unity.Bounds.Mesh();
            switch (boundsType)
            {
                case HullType.Octahedral:
                    AxisAlignedBoundingOctahedron aabo = AxisAlignedBoundingOctahedron.Reset();
                    foreach (Vector3 v in sharedMeshVertices)
                    {
                        Gizmos.DrawWireSphere(transform.TransformPoint(v), 0.1f);
                        aabo = AaboUtils.Expand(aabo, transform.TransformPoint(v));
                    }

                    Gizmos.color = blue;

                    Gizmos.DrawWireSphere(aabo.Center.xyz, 0.1f);

                    var aaboMesh = AaboUtils.GenerateMesh(aabo);
                    var numLines = aaboMesh.indices.Length/2;

                    Gizmos.color = yellow;
                    for (int i = 0; i < numLines; ++i)
                    {
                        var idx0 = aaboMesh.indices[i * 2 + 0];
                        var idx1 = aaboMesh.indices[i * 2 + 1];

                        Vector3 from = aaboMesh.vertices[idx0];
                        Vector3 to = aaboMesh.vertices[idx1];

                        Gizmos.DrawLine(from, to);
                    }

                    aaboMesh.Dispose();
                    break;
                case HullType.AABB:
                    AABB1 aabb = new AABB1();
                    foreach (Vector3 v in sharedMeshVertices)
                        aabb = AabbUtils.Expand(aabb, v);
                    //mesh = aabb.mesh;
                    break;
                case HullType.HexPrism:
                    break;
            }
        }
    }
};
