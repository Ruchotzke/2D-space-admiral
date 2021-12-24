using UnityEngine;

public static class GizmoExtensions
{
    public static void DrawSquare(Rect square, Color centerColor)
    {
        /* Draw the box */
        Gizmos.color = Color.green;
        Gizmos.DrawLine(new Vector2(square.xMin, square.yMin), new Vector2(square.xMin, square.yMax));
        Gizmos.DrawLine(new Vector2(square.xMin, square.yMax), new Vector2(square.xMax, square.yMax));
        Gizmos.DrawLine(new Vector2(square.xMax, square.yMax), new Vector2(square.xMax, square.yMin));
        Gizmos.DrawLine(new Vector2(square.xMax, square.yMin), new Vector2(square.xMin, square.yMin));

        /* Draw a cross in the center to color overlaps */
        Gizmos.color = centerColor;
        Gizmos.DrawLine(Vector2.Lerp(square.min, square.max, 0.4f), Vector2.Lerp(square.min, square.max, 0.6f));
        Gizmos.DrawLine(Vector2.Lerp(new Vector2(square.xMin, square.yMax), new Vector2(square.xMax, square.yMin), 0.4f), Vector2.Lerp(new Vector2(square.xMin, square.yMax), new Vector2(square.xMax, square.yMin), 0.6f));
    }
}
