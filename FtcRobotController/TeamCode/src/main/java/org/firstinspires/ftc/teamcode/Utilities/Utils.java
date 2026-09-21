package org.firstinspires.ftc.teamcode.Utilities;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.bezier.BezierCurve;

public final class Utils {
    private Utils() {}

    public static Pose bezierPoint(double progress, Pose... points) {
        return new BezierCurve(points).get(progress).toPose();
    }

    /**
     * Splits a Bezier curve at strictly increasing parameters in [0, 1].
     * Parameters refer to the original curve, not to each resulting segment.
     * Returns progress.length + 1 segments, each with its start, control points, and end.
     * No parameters returns the whole curve; 0 or 1 creates a degenerate endpoint segment.
     * Requires at least three points. Inputs are unchanged; output headings are zero.
     */
    public static Pose[][] splitBezierControlPoints(Pose[] points, double... progress) {
        if (points == null || points.length < 3) {
            throw new IllegalArgumentException("At least three points are required.");
        }
        double previous = -1;
        for (double split : progress) {
            if (!Double.isFinite(split) || split < 0 || split > 1 || split <= previous) {
                throw new IllegalArgumentException("Progress values must increase strictly within [0, 1].");
            }
            previous = split;
        }
        int count = points.length;
        Pose[] work = new Pose[count];
        for (int i = 0; i < count; i++) {
            work[i] = new Pose(points[i].x(), points[i].y());
        }
        Pose[][] segments = new Pose[progress.length + 1][];
        previous = 0;
        for (int segment = 0; segment < progress.length; segment++) {
            double t = (progress[segment] - previous) / (1 - previous);
            Pose[] left = new Pose[count];
            Pose[] right = new Pose[count];
            left[0] = work[0];
            right[count - 1] = work[count - 1];
            for (int level = 1; level < count; level++) {
                for (int i = 0; i < count - level; i++) {
                    work[i] = new Pose(
                            (1 - t) * work[i].x() + t * work[i + 1].x(),
                            (1 - t) * work[i].y() + t * work[i + 1].y());
                }
                left[level] = work[0];
                right[count - 1 - level] = work[count - 1 - level];
            }
            segments[segment] = left;
            work = right;
            previous = progress[segment];
        }
        segments[progress.length] = work;
        return segments;
    }
}
