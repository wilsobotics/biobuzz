package org.firstinspires.ftc.teamcode.collect;

/** Where our robot may collect (manual frame). Chosen on the gamepad during init. */
public enum ZonePreset {
    WHOLE_FIELD,
    OWN_HALF,
    NEAR_HALF,
    FAR_HALF,
    CUSTOM;

    public FieldRect rect(Alliance a) {
        switch (this) {
            case OWN_HALF:
                return FieldGeometry.ownHalf(a);
            case NEAR_HALF:
                return new FieldRect(0, 0, 144, 72);
            case FAR_HALF:
                return new FieldRect(0, 72, 144, 144);
            case CUSTOM:
                return custom();
            default:
                return FieldRect.FIELD;
        }
    }

    /** CUSTOM from CollectConfig, tolerant of swapped min/max and values off the field. */
    private static FieldRect custom() {
        double x0 = clamp(CollectConfig.CUSTOM_MIN_X, FieldRect.FIELD.minX, FieldRect.FIELD.maxX);
        double x1 = clamp(CollectConfig.CUSTOM_MAX_X, FieldRect.FIELD.minX, FieldRect.FIELD.maxX);
        double y0 = clamp(CollectConfig.CUSTOM_MIN_Y, FieldRect.FIELD.minY, FieldRect.FIELD.maxY);
        double y1 = clamp(CollectConfig.CUSTOM_MAX_Y, FieldRect.FIELD.minY, FieldRect.FIELD.maxY);
        return new FieldRect(Math.min(x0, x1), Math.min(y0, y1), Math.max(x0, x1), Math.max(y0, y1));
    }

    private static double clamp(double v, double lo, double hi) {
        return Double.isNaN(v) ? lo : Math.max(lo, Math.min(hi, v));
    }

    public ZonePreset next() {
        return values()[(ordinal() + 1) % values().length];
    }
}
