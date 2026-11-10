package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Decides whether a Limelight result becomes a frame: new, from the expected pipeline, fresh,
 * captured after the settle time, turret in place, and with a pose at its capture time.
 */
public final class FrameGate {
    public enum Reject { NONE, NO_RESULT, NOT_NEW, WRONG_PIPELINE, STALE, SETTLING, TURRET, NO_POSE }

    private long lastTimestampMs = Long.MIN_VALUE;
    private int expectedPipeline = -1;
    private long settleUntilMs = Long.MIN_VALUE;
    private long nextId = 1;
    private Reject lastReject = Reject.NO_RESULT;

    /** Call on every pipeline switch / mode change. */
    public void expect(int pipelineIndex, long nowMs, long settleMs) {
        expectedPipeline = pipelineIndex;
        settleUntilMs = nowMs + settleMs;
    }

    public Reject lastReject() {
        return lastReject;
    }

    public CameraFrame accept(RawResult r, long nowMs, CameraMode mode, PipelineSpec spec,
                              PoseHistory poses, boolean turretOk) {
        if (r == null) {
            return reject(Reject.NO_RESULT);
        }
        if (r.chTimestampMs == lastTimestampMs) {
            // Repeat poll of the same result: keep the reason it was rejected for (e.g. SETTLING)
            // so telemetry does not flicker. After an accepted frame, report NOT_NEW.
            if (lastReject == Reject.NONE || lastReject == Reject.NO_RESULT) {
                lastReject = Reject.NOT_NEW;
            }
            return null;
        }
        lastTimestampMs = r.chTimestampMs;
        if (r.pipelineIndex != expectedPipeline) {
            return reject(Reject.WRONG_PIPELINE);
        }
        if (r.stalenessMs > PerceptionConfig.MAX_STALENESS_MS) {
            return reject(Reject.STALE);
        }
        long capture = r.chTimestampMs - Math.round(r.captureLatencyMs + r.targetingLatencyMs);
        if (capture < settleUntilMs) {
            return reject(Reject.SETTLING); // judged by when the image was taken, not when it was polled
        }
        if (mode == CameraMode.BALL && !turretOk) {
            return reject(Reject.TURRET);
        }
        Pose pose = poses.at(capture);
        if (pose == null) {
            return reject(Reject.NO_POSE);
        }
        ArrayList<Detection> balls = new ArrayList<Detection>();
        if (mode == CameraMode.BALL && spec != null) {
            if (spec.type == PipelineSpec.Type.NEURAL) {
                balls.addAll(r.detections);
            } else {
                for (Detection d : r.colourBlobs) {
                    balls.add(d.withClassId(spec.colourClass.classId));
                }
            }
        }
        lastReject = Reject.NONE;
        return new CameraFrame(nextId++, mode, mode == CameraMode.BALL ? spec : null, capture, pose,
                Collections.unmodifiableList(balls));
    }

    private CameraFrame reject(Reject why) {
        lastReject = why;
        return null;
    }
}
