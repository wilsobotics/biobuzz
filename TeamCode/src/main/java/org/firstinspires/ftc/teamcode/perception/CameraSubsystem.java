package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * Sole owner of the Limelight and its tilt servo. Publishes gated, pose-stamped frames;
 * feature subsystems pull latestFrame() and skip ids they have already used.
 */
public final class CameraSubsystem implements Subsystem {
    private Limelight3A limelight;
    private Servo tilt;
    private PoseSource poseSource;
    private BooleanSupplier turretOk;          // null = no turret check, always ready
    private long turretReadySinceMs = FrameGate.TURRET_NOT_READY;
    private final PoseHistory poses = new PoseHistory(PerceptionConfig.POSE_HISTORY_MS);
    private final FrameGate gate = new FrameGate();
    private PipelineCycle cycle;
    private CameraMode mode;
    private volatile CameraFrame latestFrame;
    private volatile LLResult latestRaw;
    private volatile boolean externalPoses;

    /**
     * Fallback pose feed, sampled in periodic(). periodic() runs before the OpMode's
     * follower.update(), so these samples lag a loop; prefer recordPose().
     */
    public void setPoseSource(PoseSource source) {
        this.poseSource = source;
    }

    /**
     * Records the pose measured at tMs. Call right after follower.update() every loop (init and
     * run) so frames are paired with fresh poses. Once called, periodic() stops sampling the
     * PoseSource so the history never mixes fresh and one-loop-old poses.
     */
    public void recordPose(long tMs, Pose pose) {
        externalPoses = true;
        poses.add(tMs, pose);
    }

    /**
     * Optional: return false when the turret is not at the ball-mode angle. Evaluated every
     * periodic(); a ball frame passes only if it was captured after the check last became true.
     */
    public void setTurretCheck(BooleanSupplier check) {
        this.turretOk = check;
        turretReadySinceMs = FrameGate.TURRET_NOT_READY;
    }

    @Override
    public void initialize() {
        HardwareMap hw = ActiveOpMode.hardwareMap();
        limelight = hw.get(Limelight3A.class, PerceptionConfig.LIMELIGHT_NAME);
        tilt = PerceptionConfig.TILT_SERVO_NAME.isEmpty()
                ? null : hw.get(Servo.class, PerceptionConfig.TILT_SERVO_NAME);
        cycle = PipelineCycle.parse(PerceptionConfig.BALL_PIPELINES);
        mode = null;
        latestFrame = null;
        externalPoses = false;   // each OpMode opts in to recordPose() again
        limelight.setPollRateHz(100);
        limelight.start();
        setMode(CameraMode.BALL);
    }

    public void setMode(CameraMode m) {
        if (m == mode || limelight == null) {
            return;
        }
        mode = m;
        if (tilt != null) {
            tilt.setPosition(m == CameraMode.BALL ? PerceptionConfig.TILT_BALL_POS : PerceptionConfig.TILT_GOAL_POS);
        }
        int index = m == CameraMode.BALL ? cycle.current().index : PerceptionConfig.GOAL_PIPELINE;
        limelight.pipelineSwitch(index);
        gate.expect(index, System.currentTimeMillis(), PerceptionConfig.MODE_SETTLE_MS);
        latestFrame = null;
    }

    public Command modeCommand(final CameraMode m) {
        return instant("Camera " + m, new Runnable() {
            @Override
            public void run() {
                setMode(m);
            }
        });
    }

    @Override
    public void periodic() {
        if (limelight == null) {
            return;
        }
        long now = System.currentTimeMillis();
        if (poseSource != null && !externalPoses) {
            poses.add(now, poseSource.pose());
        }
        updateTurretReady(now);
        LLResult r = limelight.getLatestResult();
        latestRaw = r;
        if (r == null) {
            return;
        }
        PipelineSpec spec = mode == CameraMode.BALL ? cycle.current() : null;
        CameraFrame f = gate.accept(toRaw(r), now, mode, spec, poses, turretReadySinceMs);
        if (f == null) {
            return;
        }
        latestFrame = f;
        if (mode == CameraMode.BALL && cycle.size() > 1) {
            PipelineSpec next = cycle.advance();
            limelight.pipelineSwitch(next.index);
            gate.expect(next.index, now, PerceptionConfig.PIPELINE_SETTLE_MS);
        }
    }

    /** Tracks when the turret check last became true (TURRET_NOT_READY while it is false). */
    private void updateTurretReady(long now) {
        if (turretOk == null) {
            turretReadySinceMs = Long.MIN_VALUE; // no turret: every capture is after "ready"
        } else if (!turretOk.getAsBoolean()) {
            turretReadySinceMs = FrameGate.TURRET_NOT_READY;
        } else if (turretReadySinceMs == FrameGate.TURRET_NOT_READY) {
            turretReadySinceMs = now;
        }
    }

    static RawResult toRaw(LLResult r) {
        ArrayList<Detection> det = new ArrayList<Detection>();
        List<LLResultTypes.DetectorResult> dr = r.getDetectorResults();
        if (dr != null) {
            for (LLResultTypes.DetectorResult d : dr) {
                det.add(new Detection(d.getClassId(), d.getConfidence(), d.getTargetXDegrees(),
                        d.getTargetYDegrees(), Detection.aspectFromCorners(d.getTargetCorners())));
            }
        }
        ArrayList<Detection> col = new ArrayList<Detection>();
        List<LLResultTypes.ColorResult> cr = r.getColorResults();
        if (cr != null) {
            for (LLResultTypes.ColorResult c : cr) {
                col.add(new Detection(-1, 1.0, c.getTargetXDegrees(), c.getTargetYDegrees(),
                        Detection.aspectFromCorners(c.getTargetCorners())));
            }
        }
        return new RawResult(r.getControlHubTimeStamp(), r.getStaleness(), r.getPipelineIndex(),
                r.getCaptureLatency(), r.getTargetingLatency(), det, col);
    }

    public CameraMode mode() { return mode; }
    public CameraFrame latestFrame() { return latestFrame; }
    public LLResult latestRawResult() { return latestRaw; }
    public FrameGate.Reject lastReject() { return gate.lastReject(); }
}
