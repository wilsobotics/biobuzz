package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/** With no targets: look at the best hint up close, then visit known ball sources, nearest first. */
public final class SearchPlanner {
    public static final class Look {
        public final Pose pose;
        public final String key;

        Look(Pose pose, String key) {
            this.pose = pose;
            this.key = key;
        }
    }

    private final FieldFrame frame;

    public SearchPlanner(FieldFrame frame) {
        this.frame = frame;
    }

    public Look nextLook(final Pose robot, List<MapBall> hints, CollectRequest req, AllowedArea area,
                         Set<String> visited) {
        if (req.useHints) {
            ArrayList<MapBall> hs = new ArrayList<MapBall>();
            for (MapBall b : hints) {
                if (req.classes.contains(b.ballClass()) && area.targetOk(b) && !visited.contains("hint:" + b.id())) {
                    hs.add(b);
                }
            }
            Collections.sort(hs, new Comparator<MapBall>() {
                @Override
                public int compare(MapBall a, MapBall b) {
                    int q = Double.compare(b.quality(), a.quality());
                    return q != 0 ? q : Double.compare(a.distanceTo(robot.x(), robot.y()),
                            b.distanceTo(robot.x(), robot.y()));
                }
            });
            List<Zone> obstacles = FieldGeometry.staticObstacles(frame);
            for (MapBall b : hs) {
                Pose p = hintLook(robot, b, area, obstacles);
                if (p != null) {
                    return new Look(p, "hint:" + b.id());
                }
            }
        }
        Look best = null;
        double bestD = Double.MAX_VALUE;
        for (BallSource s : BallSource.ALL) {
            if (!req.sources.contains(s.name) || !s.usableBy(req.alliance) || visited.contains(s.name)) {
                continue;
            }
            double[] p = frame.toPedro(s.lookX, s.lookY);
            if (!lookOk(p[0], p[1], area, FieldGeometry.staticObstacles(frame))) {
                continue;
            }
            double d = Math.hypot(p[0] - robot.x(), p[1] - robot.y());
            if (d < bestD) {
                bestD = d;
                best = new Look(new Pose(p[0], p[1], frame.headingToPedro(Math.toRadians(s.lookHeadingDeg))), s.name);
            }
        }
        return best;
    }

    /** Standoff angles tried round a hint, nearest the direct approach first. */
    private static final int[] SWING_DEG = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};

    /**
     * A pose LOOK_DISTANCE_IN back from the hint, facing it. If the direct standoff is outside the
     * area or too close to an obstacle, the standoff swings round the hint; null if none works.
     */
    private static Pose hintLook(Pose robot, MapBall b, AllowedArea area, List<Zone> obstacles) {
        double dx = b.x() - robot.x();
        double dy = b.y() - robot.y();
        double back = Math.min(CollectConfig.LOOK_DISTANCE_IN, Math.hypot(dx, dy));
        double away = Math.atan2(-dy, -dx);   // from the hint towards the robot
        for (int deg : SWING_DEG) {
            double dir = away + Math.toRadians(deg);
            double vx = b.x() + back * Math.cos(dir);
            double vy = b.y() + back * Math.sin(dir);
            if (lookOk(vx, vy, area, obstacles)) {
                return new Pose(vx, vy, Math.atan2(b.y() - vy, b.x() - vx));
            }
        }
        return null;
    }

    /** In the area and the robot's half-length plus clearance away from every static obstacle. */
    private static boolean lookOk(double x, double y, AllowedArea area, List<Zone> obstacles) {
        if (!area.containsPedro(x, y)) {
            return false;
        }
        double margin = CollectConfig.ROBOT_LENGTH_IN / 2 + CollectConfig.CLEARANCE_IN;
        Point p = new Point(x, y);
        for (Zone z : obstacles) {
            if (z.distanceTo(p) < margin) {
                return false;
            }
        }
        return true;
    }
}
