package org.firstinspires.ftc.teamcode.collect;

import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.teamcode.perception.MapBall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Where this run may collect: the zone preset, intersected with our half in AUTO (G402). */
public final class AllowedArea {
    private final FieldRect rect;      // manual frame; null = nowhere
    private final boolean auto;
    private final FieldFrame frame;

    public AllowedArea(CollectRequest req, FieldFrame frame) {
        FieldRect r = req.zone.rect(req.alliance);
        if (req.auto) {
            r = r.intersect(FieldGeometry.ownHalf(req.alliance));
        }
        this.rect = r;
        this.auto = req.auto;
        this.frame = frame;
    }

    public FieldRect rect() {
        return rect;
    }

    public boolean containsPedro(double x, double y) {
        if (rect == null) {
            return false;
        }
        double[] m = frame.toManual(x, y);
        return rect.contains(m[0], m[1]);
    }

    /** Inside the area and, in AUTO, clear of the midline by the margin plus the ball's uncertainty. */
    public boolean targetOk(MapBall b) {
        if (!containsPedro(b.x(), b.y())) {
            return false;
        }
        if (!auto) {
            return true;
        }
        double[] m = frame.toManual(b.x(), b.y());
        return Math.abs(m[0] - FieldGeometry.FIELD_CENTER) >= CollectConfig.MIDLINE_MARGIN_IN + b.sigma();
    }

    /** The rest of the field as Weaver obstacles. */
    public List<Zone> outsideZones() {
        if (rect == null) {
            return Collections.emptyList();
        }
        ArrayList<Zone> out = new ArrayList<Zone>();
        for (FieldRect r : rect.complementIn(FieldRect.FIELD)) {
            out.add(FieldGeometry.rectZone(r, frame));
        }
        return out;
    }
}
