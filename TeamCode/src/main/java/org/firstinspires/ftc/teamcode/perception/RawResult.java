package org.firstinspires.ftc.teamcode.perception;

import java.util.List;

/** The parts of an LLResult we use, copied out so gating can be unit-tested without the SDK. */
public final class RawResult {
    public final long chTimestampMs;
    public final long stalenessMs;
    public final int pipelineIndex;
    public final double captureLatencyMs;
    public final double targetingLatencyMs;
    public final List<Detection> detections;
    public final List<Detection> colourBlobs;

    public RawResult(long chTimestampMs, long stalenessMs, int pipelineIndex, double captureLatencyMs,
                     double targetingLatencyMs, List<Detection> detections, List<Detection> colourBlobs) {
        this.chTimestampMs = chTimestampMs;
        this.stalenessMs = stalenessMs;
        this.pipelineIndex = pipelineIndex;
        this.captureLatencyMs = captureLatencyMs;
        this.targetingLatencyMs = targetingLatencyMs;
        this.detections = detections;
        this.colourBlobs = colourBlobs;
    }
}
