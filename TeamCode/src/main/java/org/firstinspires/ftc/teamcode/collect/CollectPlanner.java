package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.HeadingPolicy;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/** Chooses targets from the ball map and turns them into a Weaver route and a Pedro path. */
public final class CollectPlanner {
    public static final class Plan {
        public final Path path;
        public final List<MapBall> targets;
        public final List<Point> preview;

        Plan(Path path, List<MapBall> targets, List<Point> preview) {
            this.path = path;
            this.targets = targets;
            this.preview = preview;
        }
    }

    private final FieldFrame frame;
    private String lastError = "";

    public CollectPlanner(FieldFrame frame) {
        this.frame = frame;
    }

    /** Pushes our robot/intake geometry into Weaver's global config. */
    public static void applyWeaverConfig() {
        Weaver.setConfig(new PathConfig()
                .intakeWidth(CollectConfig.INTAKE_WIDTH_IN)
                .robotWidth(CollectConfig.ROBOT_LENGTH_IN)    // Weaver: along the forward axis
                .robotHeight(CollectConfig.ROBOT_WIDTH_IN)    // Weaver: across the forward axis
                .clearance(CollectConfig.CLEARANCE_IN));
    }

    public List<MapBall> chooseTargets(Pose robot, List<MapBall> candidates, final CollectRequest req,
                                       AllowedArea area, int capacityLeft, Set<Integer> skip) {
        int budget = capacityLeft;
        if (req.maxTargets > 0) {
            budget = Math.min(budget, req.maxTargets);
        }
        ArrayList<MapBall> pool = new ArrayList<MapBall>();
        for (MapBall b : candidates) {
            if (b.isTarget() && req.classes.contains(b.ballClass()) && !skip.contains(b.id()) && area.targetOk(b)) {
                pool.add(b);
            }
        }
        final double rx = robot.x();
        final double ry = robot.y();
        Collections.sort(pool, new Comparator<MapBall>() {
            @Override
            public int compare(MapBall a, MapBall b) {
                return Double.compare(score(a), score(b));
            }

            private double score(MapBall m) {
                return m.distanceTo(rx, ry) / req.weight(m.ballClass());
            }
        });
        ArrayList<MapBall> out = new ArrayList<MapBall>();
        int used = 0;
        MapBall maybeTwoFallback = null;   // first maybe-two that only missed for being counted as two
        for (MapBall b : pool) {
            if (used >= budget || out.size() >= CollectConfig.MAX_WEAVER_TARGETS) {
                break;
            }
            int cost = b.maybeTwo() ? 2 : 1;
            if (used + cost <= budget) {
                out.add(b);
                used += cost;
            } else if (maybeTwoFallback == null && b.maybeTwo() && budget - used >= 1) {
                maybeTwoFallback = b;
            }
        }
        // A "maybe two" may be one ball: better than an empty list.
        if (out.isEmpty() && maybeTwoFallback != null) {
            out.add(maybeTwoFallback);
        }
        return out;
    }

    /**
     * Field obstacles + the AUTO opponent-half barrier (whenever we are on our half; off it, none, so
     * we can drive home) + the preset's bands (only when we are inside the preset) + any providers.
     */
    public List<Zone> obstacles(Pose robot, AllowedArea area, List<ObstacleProvider> providers) {
        ArrayList<Zone> z = new ArrayList<Zone>(FieldGeometry.staticObstacles(frame));
        if (area.onOwnHalfPedro(robot.x(), robot.y())) {
            z.addAll(area.autoBarrierZones());
        }
        if (area.containsPedro(robot.x(), robot.y())) {
            z.addAll(area.presetBandZones());
        }
        for (ObstacleProvider p : providers) {
            z.addAll(p.obstacles());
        }
        return z;
    }

    public Plan plan(Pose robot, List<MapBall> targets, List<Zone> obstacles) {
        if (targets.isEmpty()) {
            lastError = "no targets";
            return null;
        }
        ArrayList<Point> points = new ArrayList<Point>();
        for (MapBall b : targets) {
            points.add(new Point(b.x(), b.y()));
        }
        try {
            PathResult r = Weaver.builder()
                    .start(PedroPathingConverter.toPoint(robot))
                    .targets(points)
                    .obstacles(obstacles)
                    .build();
            ArrayList<MapBall> visited = new ArrayList<MapBall>();
            ArrayList<MapBall> remaining = new ArrayList<MapBall>(targets);
            for (Point p : r.getVisitedTargets()) {
                MapBall best = null;
                double bestD = 0.5;
                for (MapBall b : remaining) {
                    double d = b.distanceTo(p.getX(), p.getY());
                    if (d <= bestD) {
                        best = b;
                        bestD = d;
                    }
                }
                if (best != null) {
                    visited.add(best);
                    remaining.remove(best);
                }
            }
            if (visited.isEmpty()) {
                lastError = "Weaver reached no targets";
                return null;
            }
            return new Plan(PedroPathingConverter.toPath(r), visited, r.getPath().sample(8));
        } catch (RuntimeException e) {
            lastError = e.toString();
            return null;
        }
    }

    /**
     * A path to a look pose, arriving with its heading. The converter makes every segment
     * tangent-heading, and Weaver's LOCKED end gets there with a sharp hook in the last segment;
     * a linear heading over the whole path turns the robot steadily from its current heading to
     * the look heading instead (fine for a holonomic drive).
     */
    public Plan planTo(Pose robot, Pose dest, List<Zone> obstacles) {
        try {
            PathResult r = Weaver.builder()
                    .start(PedroPathingConverter.toPoint(robot))
                    .end(new Point(dest.x(), dest.y(), dest.heading()), HeadingPolicy.LOCKED)
                    .obstacles(obstacles)
                    .build();
            Path path = PedroPathingConverter.toPath(r).linear(robot.heading(), dest.heading());
            return new Plan(path, Collections.<MapBall>emptyList(), r.getPath().sample(8));
        } catch (RuntimeException e) {
            lastError = e.toString();
            return null;
        }
    }

    public String lastError() {
        return lastError;
    }
}
