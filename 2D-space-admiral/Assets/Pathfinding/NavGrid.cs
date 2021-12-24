using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding2D
{
    public class NavGrid : MonoBehaviour
    {
        [Header("Scale")]
        public Rect Area;
        public int Levels;

        Quadtree tree;

        List<PathfindCell> path;

        private void Awake()
        {
            /* Generate a quadtree */
            tree = new Quadtree(Area, Levels);

            /* Generate a path along the diagonal for testing */
            GetPath(Area.min + Vector2.one, Area.max - Vector2.one);
        }

        /// <summary>
        /// Get the quadtree leaf associated with a given position.
        /// </summary>
        /// <param name="position"></param>
        /// <returns></returns>
        public Quadtree.QuadtreeNode GetNode(Vector2 position)
        {
            /* if the position is outside of the mapped area, return null */
            if (!Area.Contains(position)) return null;

            /* Recursively move down the tree */
            Quadtree.QuadtreeNode curr = tree.tree[1];
            while (!curr.isLeaf)
            {
                /* Find the correct child */
                if (tree.tree[curr.GetChild(0)].area.Contains(position))
                {
                    curr = tree.tree[curr.GetChild(0)];
                }
                else if (tree.tree[curr.GetChild(1)].area.Contains(position))
                {
                    curr = tree.tree[curr.GetChild(1)];
                }
                else if (tree.tree[curr.GetChild(2)].area.Contains(position))
                {
                    curr = tree.tree[curr.GetChild(2)];
                }
                else
                {
                    curr = tree.tree[curr.GetChild(3)];
                }
            }

            return curr;
        }

        public List<Vector2> GetPath(Vector2 startPosition, Vector2 endPosition)
        {
            /* First get the cells for the start and end */
            Quadtree.QuadtreeNode start = GetNode(startPosition);
            Quadtree.QuadtreeNode end = GetNode(endPosition);

            /* Perform an A* pathfind between the two nodes */
            List<PathfindCell> open = new List<PathfindCell>();
            List<PathfindCell> closed = new List<PathfindCell>();
            Dictionary<Quadtree.QuadtreeNode, PathfindCell> cellDict = new Dictionary<Quadtree.QuadtreeNode, PathfindCell>();

            /* Add the starting node */
            open.Add(new PathfindCell(start, end.area.center, null));
            cellDict.Add(start, open[0]);
            open[0].distance = 0.0f;

            /* Perform the A* Algorithm */
            while(open.Count > 0)
            {
                /* Get the next node */
                PathfindCell next = open[0];
                open.RemoveAt(0);
                closed.Add(next);

                /* If this node is our target, we can terminate early */
                if (next.node == end) break;

                /* Update neighbor distances */
                foreach(var neighbor in tree.GetNeighbors(next.node))
                {
                    PathfindCell neighborCell = cellDict.GetValueOrDefault(neighbor);

                    /* If the neighbor is in closed, ignore it */
                    if (closed.Contains(neighborCell)) continue;

                    /* If the neighbor is in open, update it */
                    /* Otherwise make a new pathfind node */
                    float delta = Vector2.Distance(next.node.area.center, neighbor.area.center);
                    if (neighborCell != null)
                    {
                        if(neighborCell.distance > delta + next.distance)
                        {
                            neighborCell.distance = delta + next.distance;
                            neighborCell.prev = next;
                        }
                    }
                    else
                    {
                        PathfindCell newCell = new PathfindCell(neighbor, end.area.center, next);
                        newCell.distance = next.distance + delta;
                        open.Add(newCell);
                        cellDict.Add(neighbor, newCell);
                    }
                }
            }

            /* If we finished the loop and closed does not contain the endpoint, it is not reachable */
            PathfindCell final = closed.Find(c => c.node == end);
            if(final == null)
            {
                Debug.LogError("Unable to find path.");
                return null;
            }

            /* A* Complete. Find the cell path */
            List<PathfindCell> cells = new List<PathfindCell>();
            cells.Add(final);
            while(final.node != start)
            {
                final = final.prev;
                cells.Add(final);
            }

            /* We now have a complete node path */
            Debug.Log("PATH: ");
            Debug.Log(string.Join(",\n", cells));
            path = cells;

            return null;
        }

        private void OnDrawGizmos()
        {
            if(tree != null)
            {
                /* Draw the nodes */
                foreach (var node in tree.tree.Values)
                {
                    if (node.isLeaf)
                    {
                        GizmoExtensions.DrawSquare(node.area, Color.green);
                    }
                }
            }

            /* Draw the current path */
            if (path != null)
            {
                Gizmos.color = Color.magenta;
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Gizmos.DrawLine(path[i].node.area.center, path[i+1].node.area.center);
                }
            }
        }

        private class PathfindCell : IComparable<PathfindCell>
        {
            public Quadtree.QuadtreeNode node;
            public float distance;
            public float heuristic;
            public PathfindCell prev;

            public PathfindCell(Quadtree.QuadtreeNode node, Vector2 target, PathfindCell prev = null)
            {
                this.node = node;
                this.heuristic = Vector2.Distance(node.area.center, target);
                this.prev = prev;
            }

            public int CompareTo(PathfindCell other)
            {
                return Mathf.RoundToInt((distance+heuristic) - (other.distance+other.heuristic));
            }

            public override string ToString()
            {
                return "Cell: " + node.area + " Dist: " + distance;
            }
        }
    }
}

