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
        public float Resolution;

        NavGridCell[,] Grid;
        Vector2 RectSize;
        Vector2Int Size;

        List<NavGridCell> path;
        Quadtree tree;

        List<Quadtree.QuadtreeNode> leaves = new List<Quadtree.QuadtreeNode>();
        List<Quadtree.QuadtreeNode> nodes = new List<Quadtree.QuadtreeNode>();

        private void Awake()
        {
            /* Generate a quadtree */
            tree = new Quadtree(Area, 6);
            Debug.Log(tree.tree.Keys.Count);

            /* Store the leaves */
            foreach(var node in tree.tree.Values)
            {
                if(node.isLeaf) leaves.Add(node);
            }

            /* Find a leaf node and mark its neighbors */
            nodes.Add(leaves[UnityEngine.Random.Range(0, leaves.Count)]);
            foreach (var neighbor in tree.GetNeighbors(nodes[0]))
            {
                nodes.Add(neighbor);
            }
        }

        public NavGridCell GetCell(Vector2 position)
        {
            Vector2Int index = Vector2Int.FloorToInt((position + Area.min) / RectSize);
            return Grid[index.x, index.y];
        }

        public List<NavGridCell> GetCellPath(NavGridCell start, NavGridCell end)
        {
            /* Perform an A* search to get the path */
            List<PathfindCell> Open = new List<PathfindCell>();
            List<PathfindCell> Closed = new List<PathfindCell>();

            /* Add the starter node */
            Open.Add(new PathfindCell { cell = start, distance = 0, prev = null});
            Open[0].InitializeHeuristic(end.Position);

            /* Perform A* */
            while(Open.Count > 0)
            {
                /* Pop the lowest distance item and close it*/
                PathfindCell next = Open[0];
                Open.RemoveAt(0);
                Closed.Add(next);
                //Debug.Log("Next: " + next);

                /* If next was our target, we can terminate early */
                if (next.cell == end) break;

                /* Add neighbors or update distances accordingly */
                foreach (var neighbor in GetNeighbors(next.cell))
                {
                    if (neighbor.Closed) continue;
                    PathfindCell pCell = Closed.Find(c => c.cell == neighbor);
                    if (pCell != null) continue;
                    pCell = Open.Find(c => c.cell == neighbor);
                    
                    if(pCell != null)
                    {
                        /* The cell is open, update its distance if smaller */
                        float distanceChange = Vector2.Distance(pCell.cell.Position, next.cell.Position);
                        if (pCell.distance > next.distance + distanceChange)
                        {
                            pCell.prev = next;
                            pCell.distance = next.distance + distanceChange;
                        }
                    }
                    else
                    {
                        /* This cell is unexplored. Add it */
                        Open.Add(new PathfindCell { cell = neighbor, prev = next, distance = next.distance + Vector2.Distance(neighbor.Position, next.cell.Position) });
                        Open[Open.Count - 1].InitializeHeuristic(end.Position);
                    }
                }

                /* Sort the queue for the next round */
                Open.Sort();
            }

            /* Generate a path based on the pathfind cells */
            List<NavGridCell> ret = new List<NavGridCell>();
            PathfindCell last = Closed[Closed.Count - 1];
            ret.Add(last.cell);
            while(last.cell != start)
            {
                last = last.prev;
                ret.Insert(0, last.cell);
            }

            /* Return the final cells */
            return ret;
        }
        
        /// <summary>
        /// Get the neighbors for a given cell.
        /// </summary>
        /// <param name="source"></param>
        /// <returns></returns>
        private List<NavGridCell> GetNeighbors(NavGridCell source)
        {
            List<NavGridCell> ret = new List<NavGridCell>();
            
            for(int dx = -1; dx <= 1; dx++)
            {
                for(int dy = -1; dy <= 1; dy++)
                {
                    /* Don't return the cell */
                    if (dx == 0 && dy == 0) continue;

                    /* If we are in bounds return our neighbor */
                    Vector2Int index = source.Position + new Vector2Int(dx, dy);
                    if (index.x < 0 || index.y < 0 || index.x >= Size.x || index.y >= Size.y) continue;
                    ret.Add(Grid[index.x, index.y]);
                }
            }

            return ret;
        }

        private void OnDrawGizmos()
        {
            /* Display all leaf octree nodes */
            if(tree != null)
            {
                foreach (var node in leaves)
                {
                    GizmoExtensions.DrawSquare(node.area, node.filled ? Color.red : Color.green);
                }

                /* Redraw in purple the neighbor nodes nodes */
                for (int i = 0; i < nodes.Count; i++)
                {
                    if(i == 0)
                    {
                        GizmoExtensions.DrawSquare(nodes[i].area, Color.yellow);
                    }
                    else
                    {
                        GizmoExtensions.DrawSquare(nodes[i].area, Color.magenta);
                    }
                }
            }
            
            
        }

        private class PathfindCell : IComparable<PathfindCell>
        {
            public NavGridCell cell;
            public float distance;
            public float heuristic;
            public PathfindCell prev;

            public void InitializeHeuristic(Vector2Int target)
            {
                heuristic = Vector2Int.Distance(target, cell.Position);
            }

            public int CompareTo(PathfindCell other)
            {
                return Mathf.RoundToInt((distance+heuristic) - (other.distance+other.heuristic));
            }

            public override string ToString()
            {
                return "Cell: " + cell.Position
                    + " Dist: " + distance;
            }
        }
    }

    public class NavGridCell
    {
        public Rect Area;
        public bool Closed;
        public Vector2Int Position;

        public override bool Equals(object obj)
        {
            return obj is NavGridCell cell &&
                   Area.Equals(cell.Area) &&
                   Closed == cell.Closed;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Area, Closed);
        }

        public override string ToString()
        {
            return "Cell " + Position + " Rect: " + Area;
        }
    }
}

