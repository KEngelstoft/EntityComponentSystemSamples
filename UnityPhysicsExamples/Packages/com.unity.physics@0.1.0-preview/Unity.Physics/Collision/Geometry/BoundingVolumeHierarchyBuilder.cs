﻿using System;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;
using Unity.Bounds;

namespace Unity.Physics
{
    // Utilities for building bounding volume hierarchies
    public partial struct BoundingVolumeHierarchy
    {
        public struct Constants
        {
            public const int MaxNumTreeBranches = 64;
            public const int SmallRangeSize = 32;
            public const int UnaryStackSize = 256;
            public const int BinaryStackSize = 512;
        }

        public struct PointAndIndex
        {
            public float3 Position;
            public int Index;
        }

        /// <summary>
        /// Builder.
        /// </summary>
        public unsafe struct Builder
        {
            /// <summary>
            /// Range.
            /// </summary>
            public struct Range
            {
                public Range(int start, int length, int root, AABOTetrahedra domain)
                {
                    Start = start;
                    Length = length;
                    Root = root;
                    Domain = domain;
                }

                public int Start;
                public int Length;
                public int Root;
                public AABOTetrahedra Domain;
            }

            void SortRange(int axis, ref Range range)
            {
                for (int i = range.Start; i < range.Start + range.Length; ++i)
                {
                    PointAndIndex value = Points[i];
                    float key = value.Position[axis];
                    int j = i;
                    while (j > range.Start && key < Points[j - 1].Position[axis])
                    {
                        Points[j] = Points[j - 1];
                        j--;
                    }
                    Points[j] = value;
                }
            }

            /// <summary>
            /// Compute axis and pivot of a given range.
            /// </summary>
            /// <param name="range"></param>
            /// <param name="axis"></param>
            /// <param name="pivot"></param>
            static void ComputeAxisAndPivot(ref Range range, out int axis, out float pivot)
            {
                // Compute axis and pivot.
                axis = IndexOfMaxComponent(range.Domain.Size.xyz);
                pivot = ((range.Domain.Min + range.Domain.Max) / 2)[axis];
            }

            static void SplitRange(ref Range range, int size, ref Range lRange, ref Range rRange)
            {
                lRange.Start = range.Start;
                lRange.Length = size;
                rRange.Start = lRange.Start + lRange.Length;
                rRange.Length = range.Length - lRange.Length;
            }

            struct CompareVertices : IComparer<float4>
            {
                public int Compare(float4 x, float4 y)
                {
                    return x[SortAxis].CompareTo(y[SortAxis]);
                }

                public int SortAxis;
            }

            void ProcessAxis(Range range, int axis, NativeArray<float> scores, NativeArray<float4> points, ref int bestAxis, ref int pivot, ref float minScore)
            {
                CompareVertices comparator;
                comparator.SortAxis = axis;
                points.Sort(comparator);

                PointAndIndex* p = (PointAndIndex*)PointsAsFloat4 + range.Start;

                AABOTetrahedra runningAabo = new AABOTetrahedra();
                runningAabo.Reset();

                for (int i = 0; i < points.Length; i++)
                {
                    runningAabo.Include(Aabos[p[i].Index]);
                    scores[i] = (i + 1) * Utils.CalcSurfaceArea(runningAabo);
                }

                runningAabo = new AABOTetrahedra();
                runningAabo.Reset();

                for (int i = points.Length - 1, j = 1; i > 0; --i, ++j)
                {
                    runningAabo.Include(Aabos[p[i].Index]);
                    float sum = scores[i - 1] + j * Utils.CalcSurfaceArea(runningAabo);
                    if (sum < minScore)
                    {
                        pivot = i;
                        bestAxis = axis;
                        minScore = sum;
                    }
                }
            }

            [BurstDiscard]
            void SegregateSah3(Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                NativeArray<float> scores = new NativeArray<float>(range.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<float4> pointsX = new NativeArray<float4>(range.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<float4> pointsY = new NativeArray<float4>(range.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<float4> pointsZ = new NativeArray<float4>(range.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                float4* p = PointsAsFloat4 + range.Start;

                for (int i = 0; i < range.Length; i++)
                {
                    pointsX[i] = p[i];
                }

                pointsY.CopyFrom(pointsX);
                pointsZ.CopyFrom(pointsX);

                int bestAxis = -1, pivot = -1;
                float minScore = float.MaxValue;

                ProcessAxis(range, 0, scores, pointsX, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range, 1, scores, pointsY, ref bestAxis, ref pivot, ref minScore);
                ProcessAxis(range, 2, scores, pointsZ, ref bestAxis, ref pivot, ref minScore);

                // Build sub-ranges.
                int lSize = pivot;
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);
                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                float4* sortedPoints;

                if (bestAxis == 0)
                {
                    sortedPoints = (float4*)pointsX.GetUnsafePtr();
                }
                else if (bestAxis == 1)
                {
                    sortedPoints = (float4*)pointsY.GetUnsafePtr();
                }
                else // bestAxis == 2
                {
                    sortedPoints = (float4*)pointsZ.GetUnsafePtr();
                }

                // Write back sorted points.
                for (int i = 0; i < range.Length; i++)
                {
                    p[i] = sortedPoints[i];
                }

                scores.Dispose();
                pointsX.Dispose();
                pointsY.Dispose();
                pointsZ.Dispose();
            }


            void Segregate(int axis, float pivot, Range range, int minItems, ref Range lRange, ref Range rRange)
            {
                Assert.IsTrue(range.Length > 1/*, "Range length must be greater than 1."*/);

                AABOTetrahedra lDomain = new AABOTetrahedra();
                AABOTetrahedra rDomain = new AABOTetrahedra();
                lDomain.Reset();
                rDomain.Reset();

                float4* p = PointsAsFloat4;
                float4* start = p + range.Start;
                float4* end = start + range.Length - 1;

                do
                {
                    // Consume left.

                    while (start <= end && (*start)[axis] < pivot)
                    {
                        lDomain.Include((*(start++)).xyz);
                    }

                    // Consume right.
                    while (end > start && (*end)[axis] >= pivot)
                    {
                        rDomain.Include((*(end--)).xyz);
                    }

                    if (start >= end) goto FINISHED;

                    lDomain.Include((*end).xyz);
                    rDomain.Include((*start).xyz);

                    Swap(ref *(start++), ref *(end--));
                } while (true);
            FINISHED:
                // Build sub-ranges.
                int lSize = (int)(start - p);
                int rSize = range.Length - lSize;
                if (lSize < minItems || rSize < minItems)
                {
                    // Make sure sub-ranges contains at least minItems nodes, in these rare cases (i.e. all points at the same position), we just split the set in half regardless of positions.
                    SplitRange(ref range, range.Length / 2, ref lRange, ref rRange);

                    SetAabbFromPoints(ref lDomain, PointsAsFloat4 + lRange.Start, lRange.Length);
                    //SetAabbFromPoints(ref rDomain, PointsAsFloat4 + rRange.Start, rRange.Length);

                }
                else
                {
                    SplitRange(ref range, lSize, ref lRange, ref rRange);
                }

                lRange.Domain = lDomain;
                rRange.Domain = rDomain;
            }

            void CreateChildren(Range* subRanges, int numSubRanges, int parentNodeIndex, ref int freeNodeIndex, Range* rangeStack, ref int stackSize)
            {
                int4 parentData = int4.zero;

                for (int i = 0; i < numSubRanges; i++)
                {
                    // Add child node.
                    int childNodeIndex = freeNodeIndex++;
                    parentData[i] = childNodeIndex;

                    if (subRanges[i].Length > 4)
                    {
                        // Keep splitting the range, push it on the stack.
                        rangeStack[stackSize] = subRanges[i];
                        rangeStack[stackSize++].Root = childNodeIndex;
                    }
                    else
                    {
                        Node* childNode = GetNode(childNodeIndex);
                        childNode->IsLeaf = true;

                        for (int pointIndex = 0; pointIndex < subRanges[i].Length; pointIndex++)
                        {
                            childNode->Data[pointIndex] = Points[subRanges[i].Start + pointIndex].Index;
                        }

                        for (int j = subRanges[i].Length; j < 4; j++)
                        {
                            childNode->ClearLeafData(j);
                        }
                    }
                }

                Node* parentNode = GetNode(parentNodeIndex);
                parentNode->Data = parentData;
                parentNode->IsInternal = true;
            }

            Node* GetNode(int nodeIndex) => Bvh.m_Nodes + nodeIndex;

            float4* PointsAsFloat4 => (float4*)Points.GetUnsafePtr();

            void ProcessSmallRange(Range baseRange, ref int freeNodeIndex)
            {
                Range range = baseRange;

                ComputeAxisAndPivot(ref range, out int axis, out float pivot);
                SortRange(axis, ref range);

                Range* subRanges = stackalloc Range[4];
                int hasLeftOvers = 1;
                do
                {
                    int numSubRanges = 0;
                    while (range.Length > 4 && numSubRanges < 3)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = 4;
                        numSubRanges++;

                        range.Start += 4;
                        range.Length -= 4;
                    }

                    if (range.Length > 0)
                    {
                        subRanges[numSubRanges].Start = range.Start;
                        subRanges[numSubRanges].Length = range.Length;

                        numSubRanges++;
                    }

                    hasLeftOvers = 0;
                    CreateChildren(subRanges, numSubRanges, range.Root, ref freeNodeIndex, &range, ref hasLeftOvers);

                    Assert.IsTrue(hasLeftOvers <= 1/*, "Internal error"*/);
                } while (hasLeftOvers > 0);
            }

            public void ProcessLargeRange(Range range, Range* subRanges)
            {
                if (!UseSah)
                {
                    ComputeAxisAndPivot(ref range, out int axis, out float pivot);

                    Range* temps = stackalloc Range[2];
                    Segregate(axis, pivot, range, 2, ref temps[0], ref temps[1]);

                    ComputeAxisAndPivot(ref temps[0], out int lAxis, out float lPivot);
                    Segregate(lAxis, lPivot, temps[0], 1, ref subRanges[0], ref subRanges[1]);

                    ComputeAxisAndPivot(ref temps[1], out int rAxis, out float rPivot);
                    Segregate(rAxis, rPivot, temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
                else
                {
                    Range* temps = stackalloc Range[2];
                    SegregateSah3(range, 2, ref temps[0], ref temps[1]);

                    SegregateSah3(temps[0], 1, ref subRanges[0], ref subRanges[1]);
                    SegregateSah3(temps[1], 1, ref subRanges[2], ref subRanges[3]);
                }
            }

            public void CreateInternalNodes(Range* subRanges, int numSubRanges, int root, Range* rangeStack, ref int stackSize, ref int freeNodeIndex)
            {
                int4 rootData = int4.zero;

                for (int i = 0; i < numSubRanges; ++i)
                {
                    rootData[i] = freeNodeIndex++;
                    rangeStack[stackSize] = subRanges[i];
                    rangeStack[stackSize++].Root = rootData[i];
                }

                Node* rootNode = GetNode(root);
                rootNode->Data = rootData;
                rootNode->IsInternal = true;
            }

            public void Build(Range baseRange)
            {
                Range* ranges = stackalloc Range[Constants.UnaryStackSize];
                int rangeStackSize = 1;
                ranges[0] = baseRange;

                if (baseRange.Length > 4)
                {
                    do
                    {
                        Range range = ranges[--rangeStackSize];

                        if (range.Length <= Constants.SmallRangeSize)
                        {
                            ProcessSmallRange(range, ref FreeNodeIndex);
                        }
                        else
                        {
                            Range* subRanges = stackalloc Range[4];
                            ProcessLargeRange(range, subRanges);
                            CreateChildren(subRanges, 4, range.Root, ref FreeNodeIndex, ranges, ref rangeStackSize);
                        }
                    }
                    while (rangeStackSize > 0);
                }
                else
                {
                    CreateChildren(ranges, 1, baseRange.Root, ref FreeNodeIndex, ranges, ref rangeStackSize);
                }
            }

            public BoundingVolumeHierarchy Bvh;
            public NativeArray<PointAndIndex> Points;
            public NativeArray<AABOTetrahedra> Aabos;
            public int FreeNodeIndex;
            public bool UseSah;
        }

        public unsafe JobHandle ScheduleBuildJobs(
            NativeArray<PointAndIndex> points, NativeArray<AABOTetrahedra> aabos, NativeArray<CollisionFilter> bodyFilters, NativeArray<int> shouldDoWork,
            int numThreadsHint, JobHandle inputDeps, int numNodes, NativeArray<Builder.Range> ranges, NativeArray<int> numBranches)
        {
            JobHandle handle = inputDeps;

            var branchNodeOffsets = new NativeArray<int>(Constants.MaxNumTreeBranches, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            int oldNumBranches = numBranches[0];

            // Build initial branches
            handle = new BuildFirstNLevelsJob
            {
                Points = points,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches,
                ThreadCount = numThreadsHint,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            // Build branches
            handle = new BuildBranchesJob
            {
                Points = points,
                Aabos = aabos,
                BodyFilters = bodyFilters,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                Ranges = ranges,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches
            }.ScheduleUnsafeIndex0(numBranches, 1, handle);

            // Note: This job also deallocates the aabbs and lookup arrays on completion
            handle = new FinalizeTreeJob
            {
                Aabos = aabos,
                Nodes = m_Nodes,
                NodeFilters = m_NodeFilters,
                LeafFilters = bodyFilters,
                NumNodes = numNodes,
                BranchNodeOffsets = branchNodeOffsets,
                BranchCount = numBranches,
                OldBranchCount = oldNumBranches,
                ShouldDoWork = shouldDoWork
            }.Schedule(handle);

            return handle;
        }

        public unsafe void Build(NativeArray<PointAndIndex> points, NativeArray<AABOTetrahedra> aabos, out int nodeCount, bool useSah = false)
        {
            m_Nodes[0] = Node.Empty;

            var builder = new Builder
            {
                Bvh = this,
                Points = points,
                Aabos = aabos,
                FreeNodeIndex = 2,
                UseSah = useSah
            };

            AABOTetrahedra aabo = new AABOTetrahedra();
            SetAabbFromPoints(ref aabo, (float4*)points.GetUnsafePtr(), points.Length);
            builder.Build(new Builder.Range(0, points.Length, 1, aabo));
            nodeCount = builder.FreeNodeIndex;

            Refit(aabos, 1, builder.FreeNodeIndex - 1);
        }

        // For each node between nodeStartIndex and nodeEnd index, set the collision filter info to the combination of the node's childen
        public unsafe void BuildCombinedCollisionFilter(NativeArray<CollisionFilter> leafFilterInfo, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                CollisionFilter combinationFilter = new CollisionFilter();

                if (currentNode->IsLeaf)
                {
                    // We know that at least one child will be valid, so start with that leaf's filter:
                    combinationFilter = leafFilterInfo[currentNode->Data[0]];
                    for (int j = 1; j < 4; ++j)
                    {
                        if (currentNode->IsLeafValid(j))
                        {
                            CollisionFilter leafFilter = leafFilterInfo[currentNode->Data[j]];
                            combinationFilter = CollisionFilter.CreateUnion(combinationFilter, leafFilter);
                        }
                    }
                }
                else
                {
                    combinationFilter = m_NodeFilters[currentNode->Data[0]];
                    for (int j = 1; j < 4; j++)
                    {
                        if (currentNode->IsInternalValid(j))
                        {
                            CollisionFilter nodeFilter = m_NodeFilters[currentNode->Data[j]];
                            combinationFilter = CollisionFilter.CreateUnion(combinationFilter, nodeFilter);
                        }
                    }
                }

                m_NodeFilters[i] = combinationFilter;
            }
        }

        // Set the collision filter on nodeIndex to the combination of all it's child filters. Node must not be a leaf.
        unsafe void BuildCombinedCollisionFilter(int nodeIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeIndex;

            Assert.IsTrue(currentNode->IsInternal);

            CollisionFilter combinedFilter = new CollisionFilter();
            for (int j = 0; j < 4; j++)
            {
                combinedFilter = CollisionFilter.CreateUnion(combinedFilter, m_NodeFilters[currentNode->Data[j]]);
            }

            m_NodeFilters[nodeIndex] = combinedFilter;
        }

        public unsafe void Refit(NativeArray<Aabb> aabbs, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                if (currentNode->IsLeaf)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        Aabb aabb;
                        if (currentNode->IsLeafValid(j))
                        {
                            aabb = aabbs[currentNode->Data[j]];
                        }
                        else
                        {
                            aabb =Aabb.Empty;
                        }

                        currentNode->Bounds.SetAabo(j, new AABOTetrahedra(aabb.Min, aabb.Max));
                    }
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                    {
                        AABOTetrahedra aabo = new AABOTetrahedra();
                        if (currentNode->IsInternalValid(j))
                        {
                            aabo = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabo();
                        }
                        else
                        {
                            aabo.Reset();
                        }

                        currentNode->Bounds.SetAabo(j, aabo);
                    }
                }
            }
        }

        public unsafe void Refit(NativeArray<AABOTetrahedra> aabos, int nodeStartIndex, int nodeEndIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeEndIndex;

            for (int i = nodeEndIndex; i >= nodeStartIndex; i--, currentNode--)
            {
                if (currentNode->IsLeaf)
                {
                    for (int j = 0; j < 4; ++j)
                    {
                        AABOTetrahedra aabo;
                        if (currentNode->IsLeafValid(j))
                        {
                            aabo = aabos[currentNode->Data[j]];
                        }
                        else
                        {
                            aabo = new AABOTetrahedra();
                            aabo.Reset();
                        }

                        currentNode->Bounds.SetAabo(j, aabo);
                    }
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                    {
                        AABOTetrahedra aabo = new AABOTetrahedra();
                        if (currentNode->IsInternalValid(j))
                        {
                            aabo = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabo();
                        }
                        else
                        {
                            aabo.Reset();
                        }

                        currentNode->Bounds.SetAabo(j, aabo);
                    }
                }
            }
        }

        unsafe void RefitNode(int nodeIndex)
        {
            Node* baseNode = m_Nodes;
            Node* currentNode = baseNode + nodeIndex;

            Assert.IsTrue(currentNode->IsInternal);

            for (int j = 0; j < 4; j++)
            {
                AABOTetrahedra compoundAabo = baseNode[currentNode->Data[j]].Bounds.GetCompoundAabo();
                currentNode->Bounds.SetAabo(j, compoundAabo);
            }
        }

        private struct RangeSizeAndIndex
        {
            public int RangeIndex;
            public int RangeSize;
            public int RangeFirstNodeOffset;
        }

        unsafe void SortRangeMap(RangeSizeAndIndex* rangeMap, int numElements)
        {
            for (int i = 0; i < numElements; i++)
            {
                RangeSizeAndIndex value = rangeMap[i];
                int key = rangeMap[i].RangeSize;
                int j = i;
                while (j > 0 && key > rangeMap[j - 1].RangeSize)
                {
                    rangeMap[j] = rangeMap[j - 1];
                    j--;
                }

                rangeMap[j] = value;
            }
        }

        public unsafe void BuildFirstNLevels(
            NativeArray<PointAndIndex> points,
            NativeArray<Builder.Range> branchRanges, NativeArray<int> branchNodeOffset,
            int threadCount, out int branchCount)
        {
            Builder.Range* level0 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            Builder.Range* level1 = stackalloc Builder.Range[Constants.MaxNumTreeBranches];
            int level0Size = 1;
            int level1Size = 0;

            AABOTetrahedra aabo = new AABOTetrahedra();
            SetAaboFromPoints(ref aabo, (float4*)points.GetUnsafePtr(), points.Length);
            level0[0] = new Builder.Range(0, points.Length, 1, aabo);

            int largestAllowedRange = math.max(level0[0].Length / threadCount, Constants.SmallRangeSize);
            int smallRangeThreshold = math.max(largestAllowedRange / threadCount, Constants.SmallRangeSize);
            int largestRangeInLastLevel;
            int maxNumBranchesMinusOneSplit = Constants.MaxNumTreeBranches - 3;
            int freeNodeIndex = 2;

            var builder = new Builder { Bvh = this, Points = points, UseSah = false };

            do
            {
                largestRangeInLastLevel = 0;

                for (int i = 0; i < level0Size; ++i)
                {
                    if (level0[i].Length > smallRangeThreshold && freeNodeIndex < maxNumBranchesMinusOneSplit)
                    {
                        // Split range in up to 4 sub-ranges.
                        Builder.Range* subRanges = stackalloc Builder.Range[4];

                        builder.ProcessLargeRange(level0[i], subRanges);

                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[0].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[1].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[2].Length);
                        largestRangeInLastLevel = math.max(largestRangeInLastLevel, subRanges[3].Length);

                        // Create nodes for the sub-ranges and append level 1 sub-ranges.
                        builder.CreateInternalNodes(subRanges, 4, level0[i].Root, level1, ref level1Size, ref freeNodeIndex);
                    }
                    else
                    {
                        // Too small, ignore.
                        level1[level1Size++] = level0[i];
                    }
                }

                Builder.Range* tmp = level0;
                level0 = level1;
                level1 = tmp;

                level0Size = level1Size;
                level1Size = 0;
                smallRangeThreshold = largestAllowedRange;
            } while (level0Size < Constants.MaxNumTreeBranches && largestRangeInLastLevel > largestAllowedRange);

            RangeSizeAndIndex* rangeMapBySize = stackalloc RangeSizeAndIndex[Constants.MaxNumTreeBranches];

            int nodeOffset = freeNodeIndex;
            for (int i = 0; i < level0Size; i++)
            {
                rangeMapBySize[i] = new RangeSizeAndIndex { RangeIndex = i, RangeSize = level0[i].Length, RangeFirstNodeOffset = nodeOffset };
                nodeOffset += level0[i].Length;
            }

            SortRangeMap(rangeMapBySize, level0Size);

            for (int i = 0; i < level0Size; i++)
            {
                branchRanges[i] = level0[rangeMapBySize[i].RangeIndex];
                branchNodeOffset[i] = rangeMapBySize[i].RangeFirstNodeOffset;
            }

            for (int i = level0Size; i < Constants.MaxNumTreeBranches; i++)
            {
                branchNodeOffset[i] = -1;
            }

            branchCount = level0Size;

            m_Nodes[0] = Node.Empty;
        }

        private unsafe void SetAaboFromPoints(ref AABOTetrahedra aabo, float4* points, int length)
        {
            aabo.Reset();
            for (int i = 0; i < length; i++)
            {
                aabo.Include(points[i].xyz);
            }
        }

        // Build the branch for range. Returns the index of the last built node in the range
        public int BuildBranch(NativeArray<PointAndIndex> points, NativeArray<Aabb> aabb, Builder.Range range, int firstNodeIndex)
        {
            var builder = new Builder
            {
                Bvh = this,
                Points = points,
                FreeNodeIndex = firstNodeIndex,
                UseSah = false
            };

            builder.Build(range);

            Refit(aabb, firstNodeIndex, builder.FreeNodeIndex - 1);
            RefitNode(range.Root);
            return builder.FreeNodeIndex - 1;
        }

        public int BuildBranch(NativeArray<PointAndIndex> points, NativeArray<AABOTetrahedra> aabo, Builder.Range range, int firstNodeIndex)
        {
            var builder = new Builder
            {
                Bvh = this,
                Points = points,
                FreeNodeIndex = firstNodeIndex,
                UseSah = false
            };

            builder.Build(range);

            Refit(aabo, firstNodeIndex, builder.FreeNodeIndex - 1);
            RefitNode(range.Root);
            return builder.FreeNodeIndex - 1;
        }

        // helper
        private static unsafe void SetAabbFromPoints(ref Aabb aabb, float4* points, int length)
        {
            aabb.Min = Math.Constants.Max3F;
            aabb.Max = Math.Constants.Min3F;
            for (int i = 0; i < length; i++)
            {
                aabb.Min = math.min(aabb.Min, points[i].xyz);
                aabb.Max = math.max(aabb.Max, points[i].xyz);
            }
        }

        private static unsafe void SetAabbFromPoints(ref AABOTetrahedra aabo, float4* points, int length)
        {
            aabo.Min = Math.Constants.Max4F;
            aabo.Max = Math.Constants.Min4F;
            for (int i = 0; i < length; i++)
            {
                aabo.Min = math.min(aabo.Min, points[i].xyzw);
                aabo.Max = math.max(aabo.Max, points[i].xyzw);
            }
        }

        [BurstCompile]
        public unsafe struct BuildFirstNLevelsJob : IJob
        {
            public NativeArray<PointAndIndex> Points;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            public NativeArray<Builder.Range> Ranges;
            public NativeArray<int> BranchNodeOffsets;
            public NativeArray<int> BranchCount;
            public NativeArray<int> ShouldDoWork;

            public int ThreadCount;

            public void Execute()
            {
                if (ShouldDoWork[0] == 0)
                {
                    // If we need to to skip tree building tasks, than set BranchCount to zero so
                    // that BuildBranchesJob also gets early out in runtime.
                    BranchCount[0] = 0;
                    return;
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                bvh.BuildFirstNLevels(Points, Ranges, BranchNodeOffsets, ThreadCount, out int branchCount);
                BranchCount[0] = branchCount;
            }
        }

        [BurstCompile]
        public unsafe struct BuildBranchesJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<AABOTetrahedra> Aabos;
            [ReadOnly] public NativeArray<CollisionFilter> BodyFilters;
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> BranchNodeOffsets;
            [ReadOnly] public NativeArray<int> BranchCount;

            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;

            [NativeDisableContainerSafetyRestriction]
            [DeallocateOnJobCompletion] public NativeArray<PointAndIndex> Points;

            public void Execute(int index)
            {
                Assert.IsTrue(BranchNodeOffsets[index] >= 0);
                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                int lastNode = bvh.BuildBranch(Points, Aabos, Ranges[index], BranchNodeOffsets[index]);

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(BodyFilters, BranchNodeOffsets[index], lastNode);
                    bvh.BuildCombinedCollisionFilter(Ranges[index].Root);
                }
            }
        }

        [BurstCompile]
        public unsafe struct FinalizeTreeJob : IJob
        {
            [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<AABOTetrahedra> Aabos;
            [ReadOnly] [DeallocateOnJobCompletion] public NativeArray<int> BranchNodeOffsets;
            [ReadOnly] public NativeArray<CollisionFilter> LeafFilters;
            [ReadOnly] public NativeArray<int> ShouldDoWork;
            [NativeDisableUnsafePtrRestriction]
            public Node* Nodes;
            [NativeDisableUnsafePtrRestriction]
            public CollisionFilter* NodeFilters;
            public int NumNodes;
            public int OldBranchCount;
            public NativeArray<int> BranchCount;

            public void Execute()
            {
                if (ShouldDoWork[0] == 0)
                {
                    // Restore original branch count
                    BranchCount[0] = OldBranchCount;
                    return;
                }

                int minBranchNodeIndex = BranchNodeOffsets[0] - 1;
                int branchCount = BranchCount[0];
                for (int i = 1; i < BranchCount[0]; i++)
                {
                    minBranchNodeIndex = math.min(BranchNodeOffsets[i] - 1, minBranchNodeIndex);
                }

                var bvh = new BoundingVolumeHierarchy(Nodes, NodeFilters);
                bvh.Refit(Aabos, 1, minBranchNodeIndex);

                if (NodeFilters != null)
                {
                    bvh.BuildCombinedCollisionFilter(LeafFilters, 1, minBranchNodeIndex);
                }
            }
        }

        public unsafe void CheckIntegrity(int nodeIndex = 1, int parentIndex = 0, int childIndex = 0)
        {
            Node parent = m_Nodes[parentIndex];
            Node node = m_Nodes[nodeIndex];
            AABOTetrahedra parentAabo = parent.Bounds.GetAabo(childIndex);

            for (int i = 0; i < 4; ++i)
            {
                int data = node.Data[i];
                AABOTetrahedra aabo = node.Bounds.GetAabo(i);

                bool validData = node.IsChildValid(i);

                bool validAabb = aabo.IsValid;

                if (validData != validAabb)
                {
                    throw new Exception("Invalid node should have empty AABB.");
                }

                if (validData)
                {
                    if (parentIndex != 0)
                    {
                        if (!parentAabo.Contains(aabo))
                        {
                            throw new Exception("Parent AABB do not contains child AABB");
                        }
                    }

                    if (node.IsInternal)
                    {
                        CheckIntegrity(data, nodeIndex, i);
                    }
                }
            }
        }
    }
}
