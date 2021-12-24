using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Quadtree
{
    public Dictionary<uint, QuadtreeNode> tree = new Dictionary<uint, QuadtreeNode>();

    public Quadtree(Rect area, int maxLevel)
    {
        /* Generate a quadtree */
        Queue<QuadtreeNode> open = new Queue<QuadtreeNode> ();
        open.Enqueue(new QuadtreeNode(area));

        uint levelCode = (uint)(1 << (maxLevel * 2 + 1));

        while (open.Count > 0)
        {
            /* Grab the next node and save it */
            QuadtreeNode next = open.Dequeue ();
            tree.Add(next.code, next);

            /* If this node is at the lowest level, don't generate more children */
            /* If this node was filled, generate children */
            if (next.filled && next.code < levelCode)
            {
                /* Split the node into 4 pieces */
                open.Enqueue(new QuadtreeNode(next, 0));
                open.Enqueue(new QuadtreeNode(next, 1));
                open.Enqueue(new QuadtreeNode(next, 2));
                open.Enqueue(new QuadtreeNode(next, 3));
            }

        }

    }

    QuadtreeNode GetNeighborOfGreaterOrEqualSize(QuadtreeNode node, Direction direction)
    {
        /* First get the parent of this node for usage elsewhere */
        QuadtreeNode parent = tree.GetValueOrDefault(node.GetParent());

        switch (direction)
        {
            case Direction.N:
                /* Base cases - same size */
                if(parent == null)
                {
                    /* We reached the root node. */
                    return null;
                }
                if(tree.GetValueOrDefault(parent.GetChild(0)) == node)
                {
                    /* We are the (0,0) child. Return our northern counterpart */
                    return tree[parent.GetChild(1)];
                }
                if (tree.GetValueOrDefault(parent.GetChild(2)) == node)
                {
                    /* We are the (1,0) child. Return our northern counterpart */
                    return tree[parent.GetChild(3)];
                }

                /* Larger neighbor */
                QuadtreeNode nextNode = GetNeighborOfGreaterOrEqualSize(parent, direction);
                if (nextNode == null || nextNode.isLeaf) return nextNode;

                /* We have a neighbor but that neighbor has children. Get the correct neighbor for our node */
                /* node is guaranteed to be a northern child */
                if(tree.GetValueOrDefault(parent.GetChild(1)) == node) //we are the northwestern child
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(0));
                }
                else
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(2));
                }

            case Direction.S:
                /* Base cases - same size */
                if (parent == null)
                {
                    /* We reached the root node. */
                    return null;
                }
                if (tree.GetValueOrDefault(parent.GetChild(1)) == node)
                {
                    /* We are the (0,1) child. Return our southern counterpart */
                    return tree[parent.GetChild(0)];
                }
                if (tree.GetValueOrDefault(parent.GetChild(3)) == node)
                {
                    /* We are the (1,1) child. Return our southern counterpart */
                    return tree[parent.GetChild(2)];
                }

                /* Larger neighbor */
                nextNode = GetNeighborOfGreaterOrEqualSize(parent, direction);
                if (nextNode == null || nextNode.isLeaf) return nextNode;

                /* We have a neighbor but that neighbor has children. Get the correct neighbor for our node */
                /* node is guaranteed to be a southern child */
                if (tree.GetValueOrDefault(parent.GetChild(0)) == node) //we are the southwestern child
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(1));
                }
                else
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(3));
                }

            case Direction.E:
                /* Base cases - same size */
                if (parent == null)
                {
                    /* We reached the root node. */
                    return null;
                }
                if (tree.GetValueOrDefault(parent.GetChild(0)) == node)
                {
                    /* We are the (0,0) child. Return our eastern counterpart */
                    return tree[parent.GetChild(2)];
                }
                if (tree.GetValueOrDefault(parent.GetChild(1)) == node)
                {
                    /* We are the (0,1) child. Return our eastern counterpart */
                    return tree[parent.GetChild(3)];
                }

                /* Larger neighbor */
                nextNode = GetNeighborOfGreaterOrEqualSize(parent, direction);
                if (nextNode == null || nextNode.isLeaf) return nextNode;

                /* We have a neighbor but that neighbor has children. Get the correct neighbor for our node */
                /* node is guaranteed to be a eastern child */
                if (tree.GetValueOrDefault(parent.GetChild(2)) == node) //we are the southeastern child
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(0));
                }
                else
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(1));
                }

            case Direction.W:
                /* Base cases - same size */
                if (parent == null)
                {
                    /* We reached the root node. */
                    return null;
                }
                if (tree.GetValueOrDefault(parent.GetChild(2)) == node)
                {
                    /* We are the (1,0) child. Return our western counterpart */
                    return tree[parent.GetChild(0)];
                }
                if (tree.GetValueOrDefault(parent.GetChild(3)) == node)
                {
                    /* We are the (1,1) child. Return our western counterpart */
                    return tree[parent.GetChild(1)];
                }

                /* Larger neighbor */
                nextNode = GetNeighborOfGreaterOrEqualSize(parent, direction);
                if (nextNode == null || nextNode.isLeaf) return nextNode;

                /* We have a neighbor but that neighbor has children. Get the correct neighbor for our node */
                /* node is guaranteed to be a western child */
                if (tree.GetValueOrDefault(parent.GetChild(0)) == node) //we are the southwestern child
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(2));
                }
                else
                {
                    return tree.GetValueOrDefault(nextNode.GetChild(3));
                }
            default:
                break;
        }

        Debug.Log("FALLEN THROUGH");
        return null;
    }

    List<QuadtreeNode> FindSmallerNeighbors(QuadtreeNode node, QuadtreeNode neighbor, Direction direction)
    {
        List<QuadtreeNode> candidates = new List<QuadtreeNode>();
        List<QuadtreeNode> neighbors = new List<QuadtreeNode>();

        if(neighbor != null) candidates.Add(neighbor);

        switch (direction)
        {
            case Direction.N:
                while(candidates.Count > 0)
                {
                    if (candidates[0].isLeaf)
                    {
                        neighbors.Add(candidates[0]);
                    }
                    else
                    {
                        candidates.Add(tree[candidates[0].GetChild(0)]);
                        candidates.Add(tree[candidates[0].GetChild(2)]);
                    }

                    candidates.RemoveAt(0);
                }

                return neighbors;

            case Direction.S:
                while (candidates.Count > 0)
                {
                    if (candidates[0].isLeaf)
                    {
                        neighbors.Add(candidates[0]);
                    }
                    else
                    {
                        candidates.Add(tree[candidates[0].GetChild(1)]);
                        candidates.Add(tree[candidates[0].GetChild(3)]);
                    }

                    candidates.RemoveAt(0);
                }

                return neighbors;

            case Direction.E:
                while (candidates.Count > 0)
                {
                    if (candidates[0].isLeaf)
                    {
                        neighbors.Add(candidates[0]);
                    }
                    else
                    {
                        candidates.Add(tree[candidates[0].GetChild(0)]);
                        candidates.Add(tree[candidates[0].GetChild(1)]);
                    }

                    candidates.RemoveAt(0);
                }

                return neighbors;

            case Direction.W:
                while (candidates.Count > 0)
                {
                    if (candidates[0].isLeaf)
                    {
                        neighbors.Add(candidates[0]);
                    }
                    else
                    {
                        candidates.Add(tree[candidates[0].GetChild(2)]);
                        candidates.Add(tree[candidates[0].GetChild(3)]);
                    }

                    candidates.RemoveAt(0);
                }

                return neighbors;
            default:
                break;
        }

        Debug.Log("Fall through.");
        return null;
    }

    /// <summary>
    /// Get all neighbors in the given direction.
    /// </summary>
    /// <param name="node"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    public List<QuadtreeNode> GetNeighbors(QuadtreeNode node, Direction direction)
    {
        var neighbor = GetNeighborOfGreaterOrEqualSize(node, direction);
        return FindSmallerNeighbors(node, neighbor, direction);
    }

    /// <summary>
    /// Get all neighbors for the given node in all directions (both edge and corner neighbors)
    /// </summary>
    /// <param name="node"></param>
    /// <returns></returns>
    public List<QuadtreeNode> GetNeighbors(QuadtreeNode node)
    {
        /* First get neighbors in the 4 cardinal directions */
        /* Using north and south neighbors find the corner neighbors */
        List<QuadtreeNode> ret = new List<QuadtreeNode>();
        List<QuadtreeNode> N = FindSmallerNeighbors(node, GetNeighborOfGreaterOrEqualSize(node, Direction.N), Direction.N); //northern neighbors
        List<QuadtreeNode> S = FindSmallerNeighbors(node, GetNeighborOfGreaterOrEqualSize(node, Direction.S), Direction.S); //southern neighbors
        ret.AddRange(N);
        ret.AddRange(S);
        ret.AddRange(FindSmallerNeighbors(node, GetNeighborOfGreaterOrEqualSize(node, Direction.E), Direction.E));
        ret.AddRange(FindSmallerNeighbors(node, GetNeighborOfGreaterOrEqualSize(node, Direction.W), Direction.W));

        /* TOP LEFT AND TOP RIGHT */

        /* Get the top right and top left neighbors */
        if(N.Count > 0)
        {
            var leftmost = N[0];
            var rightmost = N[0];
            for (int i = 1; i < N.Count; i++)
            {
                if (N[i].area.xMin < leftmost.area.xMin) leftmost = N[i];
                if (N[i].area.xMax > rightmost.area.xMax) rightmost = N[i];
            }

            /* Get those node's lowest neighbors (should be the corner neighbor) */
            List<QuadtreeNode> tl_neighbors = FindSmallerNeighbors(leftmost, GetNeighborOfGreaterOrEqualSize(leftmost, Direction.W), Direction.W);
            if (tl_neighbors.Count > 0)
            {
                QuadtreeNode tl_corner = tl_neighbors[0];
                for (int i = 1; i < tl_neighbors.Count; i++)
                {
                    if (tl_neighbors[i].area.yMin < tl_corner.area.yMin) tl_corner = tl_neighbors[i];
                }

                /* Only add the neighbor if it is located at the corner of the node */
                if(tl_corner.area.xMax == node.area.xMin && tl_corner.area.yMin == node.area.yMax) ret.Add(tl_corner);
            }


            List<QuadtreeNode> tr_neighbors = FindSmallerNeighbors(rightmost, GetNeighborOfGreaterOrEqualSize(rightmost, Direction.E), Direction.E);
            if (tr_neighbors.Count > 0)
            {
                QuadtreeNode tr_corner = tr_neighbors[0];
                for (int i = 1; i < tr_neighbors.Count; i++)
                {
                    if (tr_neighbors[i].area.yMin < tr_corner.area.yMin) tr_corner = tr_neighbors[i];
                }

                /* Only add the neighbor if it is located at the corner of the node */
                if (tr_corner.area.xMin == node.area.xMax && tr_corner.area.yMin == node.area.yMax) ret.Add(tr_corner);
            }
        }
        

        /* BOTTOM LEFT AND BOTTOM RIGHT */

        /* Get the top right and top left neighbors */
        if(S.Count > 0)
        {
            var leftmost = S[0];
            var rightmost = S[0];
            for (int i = 1; i < S.Count; i++)
            {
                if (S[i].area.xMin < leftmost.area.xMin) leftmost = S[i];
                if (S[i].area.xMax > rightmost.area.xMax) rightmost = S[i];
            }

            /* Get those node's highest neighbors (should be the corner neighbor) */
            List<QuadtreeNode> bl_neighbors = FindSmallerNeighbors(leftmost, GetNeighborOfGreaterOrEqualSize(leftmost, Direction.W), Direction.W);
            if (bl_neighbors.Count > 0)
            {
                QuadtreeNode bl_corner = bl_neighbors[0];
                for (int i = 1; i < bl_neighbors.Count; i++)
                {
                    if (bl_neighbors[i].area.yMin > bl_corner.area.yMin) bl_corner = bl_neighbors[i];
                }

                /* Only add the neighbor if it is located at the corner of the node */
                if (bl_corner.area.xMax == node.area.xMin && bl_corner.area.yMax == node.area.yMin) ret.Add(bl_corner);
            }

            List<QuadtreeNode> br_neighbors = FindSmallerNeighbors(rightmost, GetNeighborOfGreaterOrEqualSize(rightmost, Direction.E), Direction.E);
            if (br_neighbors.Count > 0)
            {
                QuadtreeNode br_corner = br_neighbors[0];
                for (int i = 1; i < br_neighbors.Count; i++)
                {
                    if (br_neighbors[i].area.yMin > br_corner.area.yMin) br_corner = br_neighbors[i];
                }

                /* Only add the neighbor if it is located at the corner of the node */
                if (br_corner.area.xMin == node.area.xMax && br_corner.area.yMax == node.area.yMin) ret.Add(br_corner);
            }
        }       

        /* Return all of the neighbors */
        return ret;
    }

    public class QuadtreeNode
    {
        public Rect area;
        public uint code;   /* bitwise - +/- xy i.e. 00 = lower left, 10 = lower right, etc. */
        public bool filled;
        public bool isLeaf = true;

        /// <summary>
        /// Generate the root node.
        /// </summary>
        public QuadtreeNode(Rect area)
        {
            this.area = area;
            this.code = 1;

            /* Is this node filled */
            this.filled = Physics2D.OverlapArea(area.min, area.max);
        }

        /// <summary>
        /// Generate a new child of a quadree node.
        /// </summary>
        /// <param name="parent">The parent node for this node</param>
        /// <param name="quadrant">The quadrant this node belongs to relative to its parent.</param>
        public QuadtreeNode(QuadtreeNode parent, byte quadrant)
        {
            /* Generate a code based on the parent */
            this.code = (parent.code << 2) | quadrant;

            /* Determine the correct region */
            switch (quadrant)
            {
                case 0:
                    area = new Rect(parent.area.min, parent.area.size / 2.0f);
                    break;
                case 1:
                    area = new Rect(parent.area.min + new Vector2(0f, parent.area.height / 2.0f), parent.area.size / 2.0f);
                    break;
                case 2:
                    area = new Rect(parent.area.min + new Vector2(parent.area.width / 2.0f, 0f), parent.area.size / 2.0f);
                    break;
                case 3:
                    area = new Rect(parent.area.min + parent.area.size / 2.0f, parent.area.size / 2.0f);
                    break;
                default:
                    Debug.LogError("Invalid quadrant supplied: " + quadrant);
                    break;
            }

            /* Is this node filled */
            this.filled = Physics2D.OverlapArea(area.min, area.max);

            /* Mark the parent as a non-leaf node */
            parent.isLeaf = false;
        }

        public uint GetChild(byte direction)
        {
            return code << 2 | direction;
        }

        public uint GetParent()
        {
            return code >> 2;
        }

        public override string ToString()
        {
            return "QuadCell: " + area.ToString();
        }
    }

    public enum Direction
    {
        N, S, E, W
    }
}
