package org.firstinspires.ftc.teamcode.perception;

/** BIOBUZZ scoring elements. Class ids match the Limelight model's labels.txt order. */
public enum BallClass {
    POLLEN(0, 2.8),
    RED_NECTAR(1, 3.6),
    BLUE_NECTAR(2, 3.6);

    public final int classId;
    public final double diameterIn;

    BallClass(int classId, double diameterIn) {
        this.classId = classId;
        this.diameterIn = diameterIn;
    }

    /** Height of the ball centre above the tiles, in. */
    public double centerHeightIn() {
        return diameterIn / 2.0;
    }

    /** The class for a detector class id, or null for anything that is not a ball. */
    public static BallClass fromClassId(int id) {
        for (BallClass c : values()) {
            if (c.classId == id) {
                return c;
            }
        }
        return null;
    }
}
