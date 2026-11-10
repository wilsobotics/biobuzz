package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;

import org.firstinspires.ftc.teamcode.collect.CollectBrain;
import org.firstinspires.ftc.teamcode.collect.CollectConfig;
import org.firstinspires.ftc.teamcode.collect.FieldFrame;
import org.firstinspires.ftc.teamcode.collect.FieldRect;
import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.List;

/** Draws pose, ball map, planned path and allowed area on the FTC Dashboard field (centre origin). */
public final class FieldDrawer {
    private FieldDrawer() {
    }

    public static void draw(Canvas c, Pose robot, BallMap map, CollectBrain brain, FieldFrame frame) {
        if (brain != null && brain.area().rect() != null) {
            FieldRect r = brain.area().rect();
            double[] a = dash(frame.toPedro(r.minX, r.minY));
            double[] b = dash(frame.toPedro(r.maxX, r.minY));
            double[] d = dash(frame.toPedro(r.maxX, r.maxY));
            double[] e = dash(frame.toPedro(r.minX, r.maxY));
            c.setStroke("#4CAF50").setStrokeWidth(1)
                    .strokePolygon(new double[]{a[0], b[0], d[0], e[0]}, new double[]{a[1], b[1], d[1], e[1]});
        }
        for (MapBall m : map.all()) {
            double[] p = dash(new double[]{m.x(), m.y()});
            String colour = colour(m.ballClass());
            c.setAlpha(m.isTarget() ? 1.0 : 0.4).setFill(colour).setStroke(colour);
            if (m.isConfirmed()) {
                c.fillCircle(p[0], p[1], m.ballClass().diameterIn / 2);
            } else {
                c.strokeCircle(p[0], p[1], m.ballClass().diameterIn / 2);
            }
        }
        c.setAlpha(1.0);
        if (brain != null && brain.preview().size() > 1) {
            List<Point> pts = brain.preview();
            double[] xs = new double[pts.size()];
            double[] ys = new double[pts.size()];
            for (int i = 0; i < pts.size(); i++) {
                double[] p = dash(new double[]{pts.get(i).getX(), pts.get(i).getY()});
                xs[i] = p[0];
                ys[i] = p[1];
            }
            c.setStroke("#9C27B0").setStrokeWidth(1).strokePolyline(xs, ys);
        }
        if (robot != null) {
            double[] p = dash(new double[]{robot.x(), robot.y()});
            double h = robot.heading() + Math.toRadians(CollectConfig.DASH_ROTATION_DEG);
            c.setStroke("#212121").setStrokeWidth(2).strokeCircle(p[0], p[1], 9)
                    .strokeLine(p[0], p[1], p[0] + 9 * Math.cos(h), p[1] + 9 * Math.sin(h));
        }
    }

    private static String colour(BallClass c) {
        switch (c) {
            case RED_NECTAR:
                return "#E53935";
            case BLUE_NECTAR:
                return "#1E88E5";
            default:
                return "#FBC02D";
        }
    }

    /** Pedro (corner origin) -> dashboard (centre origin), rotated by DASH_ROTATION_DEG. */
    private static double[] dash(double[] pedro) {
        double x = pedro[0] - 72;
        double y = pedro[1] - 72;
        double r = Math.toRadians(CollectConfig.DASH_ROTATION_DEG);
        return new double[]{x * Math.cos(r) - y * Math.sin(r), x * Math.sin(r) + y * Math.cos(r)};
    }
}
