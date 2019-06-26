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

            switch (boundsType)
            {
                case HullType.Octahedral:
                    {
                        var shape = new AxisAlignedBoundingOctahedron();
                        shape.Reset();

                        foreach (Vector3 v in sharedMeshVertices)
                        {
                            shape.Add(transform.TransformPoint(v));
                        }

                        var mesh = Utils.GenerateMesh(shape);
                        var numLines = mesh.indices.Length / 2;

                        Gizmos.color = yellow;
                        for (int i = 0; i < numLines; ++i)
                        {
                            var idx0 = mesh.indices[i * 2 + 0];
                            var idx1 = mesh.indices[i * 2 + 1];

                            Vector3 from = mesh.vertices[idx0];
                            Vector3 to = mesh.vertices[idx1];

                            Gizmos.DrawLine(from, to);
                        }

                        mesh.Dispose();
                    }
                    break;

                case HullType.AABB:
                    {
                        var shape = new AxisAlignedBoundingBox();
                        shape.Reset();

                        foreach (Vector3 v in sharedMeshVertices)
                        {
                            shape.Add(transform.TransformPoint(v));
                        }

                        var verts = new float3[8]
                        {
                            new float3(shape.Min.x, shape.Min.y, shape.Min.z), // 0
                            new float3(shape.Min.x, shape.Min.y, shape.Max.z), // 1
                            new float3(shape.Min.x, shape.Max.y, shape.Min.z), // 2
                            new float3(shape.Min.x, shape.Max.y, shape.Max.z), // 3
                            new float3(shape.Max.x, shape.Min.y, shape.Min.z), // 4
                            new float3(shape.Max.x, shape.Min.y, shape.Max.z), // 5
                            new float3(shape.Max.x, shape.Max.y, shape.Min.z), // 6
                            new float3(shape.Max.x, shape.Max.y, shape.Max.z), // 7
                        };

                        Gizmos.color = yellow;

                        Gizmos.DrawLine(verts[0], verts[1]);
                        Gizmos.DrawLine(verts[1], verts[3]);
                        Gizmos.DrawLine(verts[3], verts[2]);
                        Gizmos.DrawLine(verts[2], verts[0]);

                        Gizmos.DrawLine(verts[4], verts[5]);
                        Gizmos.DrawLine(verts[5], verts[7]);
                        Gizmos.DrawLine(verts[7], verts[6]);
                        Gizmos.DrawLine(verts[6], verts[4]);

                        Gizmos.DrawLine(verts[0], verts[4]);
                        Gizmos.DrawLine(verts[1], verts[5]);
                        Gizmos.DrawLine(verts[2], verts[6]);
                        Gizmos.DrawLine(verts[3], verts[7]);
                    }
                    break;

                case HullType.HexPrism:
                    {
                        var shape = new AxisAlignedBoundingHexagonalPrism();
                        shape.Reset();

                        foreach (Vector3 v in sharedMeshVertices)
                        {
                            shape.Add(transform.TransformPoint(v));
                        }

                        var mesh = Utils.GenerateMesh(shape);
                        var numLines = mesh.indices.Length / 2;

                        Gizmos.color = yellow;
                        for (int i = 0; i < numLines; ++i)
                        {
                            var idx0 = mesh.indices[i * 2 + 0];
                            var idx1 = mesh.indices[i * 2 + 1];

                            Vector3 from = mesh.vertices[idx0];
                            Vector3 to = mesh.vertices[idx1];

                            Gizmos.DrawLine(from, to);
                        }

                        mesh.Dispose();
                    }
                    break;
            }
        }
    }
};
