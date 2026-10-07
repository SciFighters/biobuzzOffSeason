package org.firstinspires.ftc.teamcode.Utilities;

import static com.pedropathing.api.Paths.through;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.tour.HeldKarpTSP;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;

import java.util.List;

public final class HeldKarpPath {
    public static final int maxElements = 4;

    public static Path plan(Pose start, List<Pose> targets) {
        if (targets.isEmpty()) return null;

        Pose[] points = new Pose[Math.min(targets.size(), maxElements) + 1];
        points[0] = start;
        for (int i = 1; i < points.length; i++) {
            points[i] = targets.get(i - 1);
        }

        Graph<Integer, DefaultWeightedEdge> graph =
                new DefaultDirectedWeightedGraph<>(DefaultWeightedEdge.class);
        for (int i = 0; i < points.length; i++) {
            graph.addVertex(i);
        }

        for (int from = 0; from < points.length; from++) {
            Pose origin = points[from];
            for (int to = from + 1; to < points.length; to++) {
                Pose target = points[to];
                double distance = Math.hypot(target.x() - origin.x(), target.y() - origin.y());

                DefaultWeightedEdge edge = graph.addEdge(from, to);
                graph.setEdgeWeight(edge, distance);
                DefaultWeightedEdge reverseEdge = graph.addEdge(to, from);
                graph.setEdgeWeight(reverseEdge, from == 0 ? 0 : distance);
            }
        }

        GraphPath<Integer, DefaultWeightedEdge> tour =
                new HeldKarpTSP<Integer, DefaultWeightedEdge>().getTour(graph);
        List<Integer> cycle = tour.getVertexList();
        int cycleSize = cycle.size() - 1;
        int startIndex = cycle.indexOf(0);

        Pose[] orderedPoints = new Pose[cycleSize];
        orderedPoints[0] = start;
        for (int offset = 1; offset < cycleSize; offset++) {
            int targetIndex = cycle.get((startIndex + offset) % cycleSize);
            orderedPoints[offset] = points[targetIndex];
        }
        return through(orderedPoints).tangent();
    }
}
