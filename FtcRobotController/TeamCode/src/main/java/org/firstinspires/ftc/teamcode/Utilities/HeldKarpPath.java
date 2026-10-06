package org.firstinspires.ftc.teamcode.Utilities;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.tour.HeldKarpTSP;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.DefaultWeightedEdge;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class HeldKarpPath {
    public static final int maxElements = 4;

    public static Path plan(Pose start, List<Pose> targets) {
        List<Pose> orderedTargets = visitOrder(start, targets);
        if (orderedTargets.isEmpty()) {
            throw new IllegalArgumentException("at least one element required");
        }
        List<Path> segments = new ArrayList<>();
        for (Pose target : orderedTargets) {
            target = target.withHeading(
                    Math.atan2(target.y() - start.y(), target.x() - start.x()));
            segments.add(line(start, target).linear(start.heading(), target.heading()));
            start = target;
        }
        return path(segments.toArray(new Path[0]));
    }

    public static List<Pose> visitOrder(Pose start, List<Pose> targets) {
        if (targets.size() > maxElements) {
            throw new IllegalArgumentException("too many elements");
        }
        if (targets.isEmpty()) {
            return Collections.emptyList();
        }

        List<Pose> points = new ArrayList<>();
        points.add(start);
        points.addAll(targets);
        Graph<Integer, DefaultWeightedEdge> graph =
                new DefaultDirectedWeightedGraph<>(DefaultWeightedEdge.class);
        for (int i = 0; i < points.size(); i++) {
            graph.addVertex(i);
        }
        for (int from = 0; from < points.size(); from++) {
            for (int to = 0; to < points.size(); to++) {
                if (from != to) {
                    Pose origin = points.get(from);
                    Pose target = points.get(to);
                    double distance = Math.hypot(target.x() - origin.x(), target.y() - origin.y());
                    if (distance == 0) {
                        throw new IllegalArgumentException("Positions must be distinct.");
                    }
                    DefaultWeightedEdge edge = graph.addEdge(from, to);
                    graph.setEdgeWeight(edge, to == 0 ? 0 : distance);
                }
            }
        }
        GraphPath<Integer, DefaultWeightedEdge> tour =
                new HeldKarpTSP<Integer, DefaultWeightedEdge>().getTour(graph);
        List<Integer> cycle = tour.getVertexList();
        int cycleSize = cycle.size() - 1;
        int startIndex = cycle.indexOf(0);
        List<Pose> ordered = new ArrayList<>();
        for (int offset = 1; offset < cycleSize; offset++) {
            ordered.add(points.get(cycle.get((startIndex + offset) % cycleSize)));
        }
        return Collections.unmodifiableList(ordered);
    }
}
