package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Decides whether a Limelight result becomes a frame: new, from the expected pipeline, fresh,
 * captured after the settle time and after the turret reached the ball angle, and with a pose at
 * its capture time. A result captured after the newest recorded pose is not consumed: it is
 * retried (POSE_PENDING) until a pose brackets it, or for at most POSE_WAIT_MAX_MS, after which
 * it is accepted with the newest pose.
 */
public final class FrameGate {
    public enum Reject { NONE, NO_RESULT, NOT_NEW, WRONG_PIPELINE, STALE, SETTLING, TURRET, NO_POSE, POSE_PENDING }

    /** turretReadySinceMs value meaning the turret is not at the ball-mode angle. */
    public static final long TURRET_NOT_READY = -1;

    private long lastTimestampMs = Long.MIN_VALUE;
    private int expectedPipeline = -1;
    private long settleUntilMs = Long.MIN_VALUE;
    private long nextId = 1;
    private Reject lastReject = Reject.NO_RESULT;
    private long pendingTimestampMs = Long.MIN_VALUE;
    private long pendingSinceMs;

    /** Call on every pipeline switch / mode change. */
    public void expect(int pipelineIndex, long nowMs, long settleMs) {
        expectedPipeline = pipelineIndex;
        settleUntilMs = nowMs + settleMs;
    }

    public Reject lastReject() {
        return lastReject;
    }

    /**
     * @param turretReadySinceMs when the turret last reached the ball-mode angle, or
     *                           TURRET_NOT_READY; ball frames captured before it are rejected.
     */
    public CameraFrame accept(RawResult r, long nowMs, CameraMode mode, PipelineSpec spec,
                              PoseHistory poses, long turretReadySinceMs) {
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
        long previousTimestampMs = lastTimestampMs;
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
        if (mode == CameraMode.BALL
                && (turretReadySinceMs == TURRET_NOT_READY || capture < turretReadySinceMs)) {
            return reject(Reject.TURRET); // judged by capture time: a late frame may predate the turret
        }
        long newest = poses.newestTime();
        if (newest != Long.MIN_VALUE && capture > newest) {
            if (pendingTimestampMs != r.chTimestampMs) {
                pendingTimestampMs = r.chTimestampMs;
                pendingSinceMs = nowMs;
            }
            if (nowMs - pendingSinceMs < PerceptionConfig.POSE_WAIT_MAX_MS) {
                lastTimestampMs = previousTimestampMs; // not consumed: retry this result next loop
                return reject(Reject.POSE_PENDING);
            }
            // waited long enough: fall through and use the newest pose
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
