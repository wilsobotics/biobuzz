package org.firstinspires.ftc.teamcode.collect;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Known places balls sit (manual section 10), with a pose to look from (manual frame, heading in
 * degrees CCW from +x). Adding a spot = one line in ALL.
 */
public final class BallSource {
    public enum Kind { GARDEN, LOADING_ZONE, FLOWER, CUSTOM }

    public final String name;
    public final Kind kind;
    public final Alliance alliance;   // null = either alliance
    public final double lookX;
    public final double lookY;
    public final double lookHeadingDeg;
    public final boolean enabledByDefault;

    public BallSource(String name, Kind kind, Alliance alliance, double lookX, double lookY,
                      double lookHeadingDeg, boolean enabledByDefault) {
        this.name = name;
        this.kind = kind;
        this.alliance = alliance;
        this.lookX = lookX;
        this.lookY = lookY;
        this.lookHeadingDeg = lookHeadingDeg;
        this.enabledByDefault = enabledByDefault;
    }

    public boolean usableBy(Alliance a) {
        return alliance == null || alliance == a;
    }

    /** Gardens are scoring areas, so they're off by default; flowers stay off until the intake can use the retrieval opening. */
    public static final List<BallSource> ALL = Collections.unmodifiableList(Arrays.asList(
            new BallSource("GARDEN_RED", Kind.GARDEN, Alliance.RED, 14, 32, -90, false),
            new BallSource("GARDEN_BLUE", Kind.GARDEN, Alliance.BLUE, 130, 112, 90, false),
            new BallSource("LOADING_ZONE_RED", Kind.LOADING_ZONE, Alliance.RED, 36, 108, 180, true),
            new BallSource("LOADING_ZONE_BLUE", Kind.LOADING_ZONE, Alliance.BLUE, 108, 36, 0, true),
            new BallSource("FLOWER_1", Kind.FLOWER, null, 48, 120, 90, false),
            new BallSource("FLOWER_2", Kind.FLOWER, null, 96, 24, -90, false),
            new BallSource("FLOWER_3", Kind.FLOWER, null, 24, 48, 180, false),
            new BallSource("FLOWER_4", Kind.FLOWER, null, 120, 96, 0, false)));
}
