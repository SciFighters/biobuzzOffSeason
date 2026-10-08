package org.firstinspires.ftc.teamcode.Utilities;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.subSystems.IntakeCameraSubsystem;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public final class BruteForcePath {
    public static final int maxElements = 4;

    private BruteForcePath() {}

    public static Path plan(IntakeCameraSubsystem camera, Follower follower,
                            ColorRange color, Pose controlPoint) {
        Pose start = follower.pose();
        List<Pose> targets = new ArrayList<>();
        for (Rect blob : camera.scanBlobsByDistance(color)) {
            if (targets.size() >= maxElements) break;
            double[] location = camera.getElementLocation(follower, blob);
            targets.add(new Pose(location[0], location[1]));
        }
        return plan(start, targets, controlPoint);
    }

    public static Path plan(Pose start, List<Pose> targets, Pose controlPoint) {
        if (targets.isEmpty()) return null;

        Pose[] points = new Pose[Math.min(targets.size(), maxElements) + 1];
        points[0] = start;
        for (int i = 1; i < points.length; i++) {
            points[i] = targets.get(i - 1);
        }

        Pose[] bestOrder = findBestOrder(points, 1, controlPoint);
        Path[] segments = new Path[bestOrder.length - 1];
        for (int i = 0; i < segments.length; i++) {
            segments[i] = line(bestOrder[i], bestOrder[i + 1]);
        }
        return path(segments).tangent();
    }
    private static double score(Pose[] points, Pose controlPoint) {
        double total = 0;
        for (int i = 1; i < points.length; i++) {
            total += points[i - 1].distance(points[i]);
        }
        return total + (controlPoint == null ? 0 : points[points.length - 1].distance(controlPoint));
    }

    private static Pose[] findBestOrder(Pose[] points, int index, Pose controlPoint) {
        Pose[] best = points;
        for (int i = index; i < points.length; i++) {
            Pose[] order = points.clone();
            order[index] = points[i];
            order[i] = points[index];
            order = findBestOrder(order, index + 1, controlPoint);

            double currentScore = score(order, controlPoint);
            double bestScore = score(best, controlPoint);
            int last = points.length - 1;
            if (currentScore < bestScore || (controlPoint != null && currentScore == bestScore
                    && order[last].distance(controlPoint) < best[last].distance(controlPoint))) {
                best = order;
            }
        }
        return best;
    }
}
