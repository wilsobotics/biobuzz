package org.firstinspires.ftc.teamcode.perception;

/** One ball in the field-frame map. Mutated only by BallMap. */
public final class MapBall {
    private final int id;
    private final BallClass ballClass;
    double x;
    double y;
    double sigma;
    double quality;
    int seenCount;
    long lastSeenMs;
    int missFrames;
    boolean maybeTwo;
    /** Sharpest sigma among sightings that set maybeTwo (not the fused sigma); never increases. */
    double shapeSigma;

    MapBall(int id, Sighting s, long nowMs) {
        this.id = id;
        this.ballClass = s.ballClass;
        this.x = s.x;
        this.y = s.y;
        this.sigma = Math.max(PerceptionConfig.MIN_SIGMA_IN, s.sigma);
        this.quality = s.quality;
        this.seenCount = 1;
        this.lastSeenMs = nowMs;
        this.maybeTwo = s.maybeTwo;
        this.shapeSigma = Math.max(PerceptionConfig.MIN_SIGMA_IN, s.sigma);
    }

    public int id() { return id; }
    public BallClass ballClass() { return ballClass; }
    public double x() { return x; }
    public double y() { return y; }
    public double sigma() { return sigma; }
    public double quality() { return quality; }
    public int seenCount() { return seenCount; }
    public long lastSeenMs() { return lastSeenMs; }
    public boolean maybeTwo() { return maybeTwo; }

    public boolean isConfirmed() {
        return seenCount >= PerceptionConfig.CONFIRM_FRAMES;
    }

    /** Good enough to drive to. */
    public boolean isTarget() {
        return isConfirmed() && quality >= PerceptionConfig.TARGET_QUALITY && isPrecise();
    }

    /** Seen, but too far/uncertain to drive to: worth a closer look. */
    public boolean isHint() {
        return isConfirmed() && !isTarget();
    }

    boolean isPrecise() {
        return sigma <= PerceptionConfig.TARGET_SIGMA_IN;
    }

    public double distanceTo(double px, double py) {
        return Math.hypot(x - px, y - py);
    }
}
