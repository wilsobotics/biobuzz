package org.firstinspires.ftc.teamcode.collect;

import java.util.ArrayList;
import java.util.List;

/** Axis-aligned rectangle in the manual frame (inches). */
public final class FieldRect {
    public static final FieldRect FIELD = new FieldRect(0, 0, 144, 144);

    public final double minX;
    public final double minY;
    public final double maxX;
    public final double maxY;

    public FieldRect(double minX, double minY, double maxX, double maxY) {
        if (maxX < minX || maxY < minY) {
            throw new IllegalArgumentException("Empty rectangle");
        }
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
    }

    public boolean contains(double x, double y) {
        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Overlap, or null if the rectangles don't overlap. */
    public FieldRect intersect(FieldRect o) {
        double a = Math.max(minX, o.minX);
        double b = Math.max(minY, o.minY);
        double c = Math.min(maxX, o.maxX);
        double d = Math.min(maxY, o.maxY);
        if (a > c || b > d) {
            return null;
        }
        return new FieldRect(a, b, c, d);
    }

    /** The parts of outer that are not this rectangle, as up to four bands. */
    public List<FieldRect> complementIn(FieldRect outer) {
        ArrayList<FieldRect> out = new ArrayList<FieldRect>();
        if (minX > outer.minX) {
            out.add(new FieldRect(outer.minX, outer.minY, minX, outer.maxY));
        }
        if (maxX < outer.maxX) {
            out.add(new FieldRect(maxX, outer.minY, outer.maxX, outer.maxY));
        }
        if (minY > outer.minY) {
            out.add(new FieldRect(minX, outer.minY, maxX, minY));
        }
        if (maxY < outer.maxY) {
            out.add(new FieldRect(minX, maxY, maxX, outer.maxY));
        }
        return out;
    }
}
