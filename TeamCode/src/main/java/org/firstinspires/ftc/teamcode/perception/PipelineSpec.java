package org.firstinspires.ftc.teamcode.perception;

/** One Limelight ball pipeline. A NEURAL pipeline reports all classes; a COLOUR pipeline tracks one class. */
public final class PipelineSpec {
    public enum Type { NEURAL, COLOUR }

    public final int index;
    public final Type type;
    public final BallClass colourClass;

    private PipelineSpec(int index, Type type, BallClass colourClass) {
        this.index = index;
        this.type = type;
        this.colourClass = colourClass;
    }

    public static PipelineSpec neural(int index) {
        return new PipelineSpec(index, Type.NEURAL, null);
    }

    public static PipelineSpec colour(int index, BallClass c) {
        return new PipelineSpec(index, Type.COLOUR, c);
    }
}
