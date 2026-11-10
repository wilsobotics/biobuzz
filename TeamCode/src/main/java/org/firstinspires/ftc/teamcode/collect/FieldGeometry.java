package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;

import java.util.ArrayList;
import java.util.List;

/**
 * BIOBUZZ field geometry in the manual frame (manual TU08 section 9, Fig. 9-2/9-4).
 * Positions are read off the drawings: verify on a real field.
 */
public final class FieldGeometry {
    private FieldGeometry() {
    }

    public static final double FIELD_CENTER = 72.0;
    /** Hive frame base, 49.46 x 38.95 in, centred. */
    public static final FieldRect HIVE = new FieldRect(47.0, 52.5, 97.0, 91.5);
    /** Flower centres on the perimeter walls. */
    public static final double[][] FLOWERS = {{48, 144}, {96, 0}, {0, 48}, {144, 96}};
    public static final double FLOWER_RADIUS_IN = 4.0;

    /** AUTO sides (manual G402): columns A-C red, D-F blue. */
    public static FieldRect ownHalf(Alliance a) {
        return a == Alliance.RED ? new FieldRect(0, 0, 72, 144) : new FieldRect(72, 0, 144, 144);
    }

    public static List<Zone> staticObstacles(FieldFrame f) {
        ArrayList<Zone> out = new ArrayList<Zone>();
        out.add(rectZone(HIVE, f));
        for (double[] c : FLOWERS) {
            double[] p = f.toPedro(c[0], c[1]);
            out.add(new CircleZone(new Point(p[0], p[1]), FLOWER_RADIUS_IN));
        }
        return out;
    }

    public static Zone rectZone(FieldRect r, FieldFrame f) {
        double[] a = f.toPedro(r.minX, r.minY);
        double[] b = f.toPedro(r.maxX, r.minY);
        double[] c = f.toPedro(r.maxX, r.maxY);
        double[] d = f.toPedro(r.minX, r.maxY);
        return new PolygonZone(new Point(a[0], a[1]), new Point(b[0], b[1]),
                new Point(c[0], c[1]), new Point(d[0], d[1]));
    }
}
