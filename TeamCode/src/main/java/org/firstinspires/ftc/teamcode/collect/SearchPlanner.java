package org.firstinspires.ftc.teamcode.collect;

import com.pedropathing.math.Pose;

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
            for (MapBall b : hs) {
                double dx = b.x() - robot.x();
                double dy = b.y() - robot.y();
                double d = Math.hypot(dx, dy);
                double heading = Math.atan2(dy, dx);
                double back = Math.min(CollectConfig.LOOK_DISTANCE_IN, d);
                double vx = b.x() - back * Math.cos(heading);
                double vy = b.y() - back * Math.sin(heading);
                if (area.containsPedro(vx, vy)) {
                    return new Look(new Pose(vx, vy, heading), "hint:" + b.id());
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
            if (!area.containsPedro(p[0], p[1])) {
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
}
