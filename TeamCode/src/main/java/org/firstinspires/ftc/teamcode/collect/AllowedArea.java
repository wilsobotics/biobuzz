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
    private final FieldRect ownHalf;

    public AllowedArea(CollectRequest req, FieldFrame frame) {
        FieldRect r = req.zone.rect(req.alliance);
        if (req.auto) {
            r = r.intersect(FieldGeometry.ownHalf(req.alliance));
        }
        this.rect = r;
        this.auto = req.auto;
        this.frame = frame;
        this.ownHalf = FieldGeometry.ownHalf(req.alliance);
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

    /** In AUTO, whether a Pedro point is on our half (G402); always true outside AUTO. */
    public boolean onOwnHalfPedro(double x, double y) {
        if (!auto) {
            return true;
        }
        double[] m = frame.toManual(x, y);
        return ownHalf.contains(m[0], m[1]);
    }

    /**
     * The mandatory AUTO barrier: the opponent's half as one Weaver obstacle (crossing is a MAJOR
     * FOUL, G402). Empty outside AUTO. Independent of the zone preset.
     */
    public List<Zone> autoBarrierZones() {
        if (!auto) {
            return Collections.emptyList();
        }
        ArrayList<Zone> out = new ArrayList<Zone>();
        for (FieldRect r : ownHalf.complementIn(FieldRect.FIELD)) {
            out.add(FieldGeometry.rectZone(r, frame));
        }
        return out;
    }

    /** The preset's own limits: the parts of our half (AUTO) or the field (TeleOp) outside the area. */
    public List<Zone> presetBandZones() {
        if (rect == null) {
            return Collections.emptyList();
        }
        ArrayList<Zone> out = new ArrayList<Zone>();
        for (FieldRect r : rect.complementIn(bounds())) {
            out.add(FieldGeometry.rectZone(r, frame));
        }
        return out;
    }

    /** The rest of the field as Weaver obstacles: the AUTO barrier plus the preset bands. */
    public List<Zone> outsideZones() {
        ArrayList<Zone> out = new ArrayList<Zone>(autoBarrierZones());
        out.addAll(presetBandZones());
        return out;
    }

    private FieldRect bounds() {
        return auto ? ownHalf : FieldRect.FIELD;
    }
}
