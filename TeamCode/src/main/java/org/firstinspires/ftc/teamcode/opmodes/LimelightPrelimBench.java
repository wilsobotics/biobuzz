/*
 * LimelightPrelimBench - on-robot OpMode for the prelim camera session.
 * Moves the Limelight tilt servo, switches between the ball pipeline and the
 * goal (AprilTag) pipeline, shows what the camera sees, times mode switches
 * and logs every fresh result to a CSV. Ported from Hive Vision's bench and
 * wired to PerceptionConfig / BallProjector, so its numbers are the ones the
 * perception code uses.
 *
 *   D-pad UP/DOWN      tilt servo +/-0.01 (hold RIGHT BUMPER for +/-0.10)
 *   D-pad RIGHT/LEFT   this mode's assumed pitch +/-0.5 deg (deg BELOW
 *                      horizontal, negative = up). Drives the distance readout.
 *   A / Y              BALL mode / GOAL mode (pipeline + that mode's setpoint)
 *   X                  save the current servo position and pitch for this mode
 *                      into PerceptionConfig (live until the app restarts; copy
 *                      the logged values into PerceptionConfig.java to keep them)
 *   B                  MARK: next mark number in the CSV + a Limelight snapshot
 *   LEFT STICK BUTTON  switch-timing test: SWITCH_TRIALS ball<->goal switches.
 *                      Needs a ball in the ball view and a tag in the goal view.
 *   RIGHT STICK BUTTON start/stop a snapshot clip: a raw frame saved on the
 *                      Limelight every SNAPSHOT_PERIOD_MS (capture sessions)
 *
 * Works without the servo (rig or fixed mount): set the angle by hand and
 * enter it with D-pad RIGHT/LEFT. CSV lands in FIRST/perception/ on the hub.
 */
package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallProjector;
import org.firstinspires.ftc.teamcode.perception.CameraMount;
import org.firstinspires.ftc.teamcode.perception.PerceptionConfig;
import org.firstinspires.ftc.teamcode.perception.PipelineCycle;
import org.firstinspires.ftc.teamcode.perception.PipelineSpec;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Limelight Prelim Bench", group = "Perception")
public class LimelightPrelimBench extends LinearOpMode {

    private static final double SERVO_STEP = 0.01;
    private static final double SERVO_STEP_BIG = 0.10;
    private static final double PITCH_STEP_DEG = 0.5;
    private static final int    SWITCH_TRIALS = 10;
    private static final long   SWITCH_TIMEOUT_MS = 3000;
    private static final long   SWITCH_PAUSE_MS = 500;    // hold each mode this long between trials
    private static final long   SNAPSHOT_PERIOD_MS = 500; // 2 frames/s, the capture plan's sampling rate
    private static final int    MAX_SHOWN = 6;            // targets listed in telemetry

    private enum Mode { BALL, GOAL }

    private Limelight3A limelight;
    private Servo tilt;                                   // null = no servo wired
    private PrintWriter csv;                              // null = logging failed (shown in telemetry)
    private String csvStatus;
    private PipelineSpec ballPipeline;                    // first entry of PerceptionConfig.BALL_PIPELINES

    private Mode mode = Mode.BALL;
    private double servoPos;
    private final double[] setpoint = {PerceptionConfig.TILT_BALL_POS, PerceptionConfig.TILT_GOAL_POS};
    private final double[] pitchDeg = {PerceptionConfig.BALL_PITCH_DOWN_DEG, -PerceptionConfig.BALL_PITCH_DOWN_DEG};

    private int mark = 0;
    private int clip = 0;
    private int clipFrame = 0;
    private boolean clipping = false;
    private long lastSnapMs = 0;
    private long lastResultStamp = -1;
    private final List<Long> recentStamps = new ArrayList<Long>();
    private String switchSummary = "not run";

    private final boolean[] prev = new boolean[12];

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, PerceptionConfig.LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        tilt = PerceptionConfig.TILT_SERVO_NAME.isEmpty() ? null
                : hardwareMap.tryGet(Servo.class, PerceptionConfig.TILT_SERVO_NAME);
        ballPipeline = PipelineCycle.parse(PerceptionConfig.BALL_PIPELINES).current();
        openCsv();

        telemetry.addLine("Limelight Prelim Bench");
        telemetry.addLine(tilt == null ? "No tilt servo '" + PerceptionConfig.TILT_SERVO_NAME + "': set the angle by hand."
                : "Tilt servo found: it moves to the ball setpoint on START.");
        telemetry.addData("Pipelines", "ball %d (%s)  goal %d", ballPipeline.index, ballPipeline.type,
                PerceptionConfig.GOAL_PIPELINE);
        telemetry.addLine(csvStatus);
        telemetry.update();
        waitForStart();

        limelight.start();
        enter(Mode.BALL);
        try {
            while (opModeIsActive()) {
                handleButtons();
                LLResult r = limelight.getLatestResult();
                // After A/Y the Limelight can still hand back the old pipeline's frames for a moment:
                // never log or show those under the new mode.
                if (r != null && r.getPipelineIndex() != pipelineFor(mode)) {
                    r = null;
                }
                boolean fresh = r != null && r.getControlHubTimeStamp() != lastResultStamp;
                if (fresh) {
                    lastResultStamp = r.getControlHubTimeStamp();
                    countFps(lastResultStamp);
                    logResult(r);
                }
                snapshotClipFrame();
                show(r);
            }
        } finally {
            limelight.stop();
            if (csv != null) {
                csv.close();
            }
        }
    }

    /* ---------------- controls ---------------- */

    private boolean pressed(int i, boolean now) {
        boolean edge = now && !prev[i];
        prev[i] = now;
        return edge;
    }

    private void handleButtons() {
        double step = gamepad1.right_bumper ? SERVO_STEP_BIG : SERVO_STEP;
        if (pressed(0, gamepad1.dpad_up)) {
            setServo(servoPos + step);
        }
        if (pressed(1, gamepad1.dpad_down)) {
            setServo(servoPos - step);
        }
        if (pressed(2, gamepad1.dpad_right)) {
            pitchDeg[mode.ordinal()] += PITCH_STEP_DEG;
        }
        if (pressed(3, gamepad1.dpad_left)) {
            pitchDeg[mode.ordinal()] -= PITCH_STEP_DEG;
        }
        if (pressed(4, gamepad1.a)) {
            enter(Mode.BALL);
        }
        if (pressed(5, gamepad1.y)) {
            enter(Mode.GOAL);
        }
        if (pressed(6, gamepad1.x)) {
            saveSetpoint();
        }
        if (pressed(7, gamepad1.b)) {
            mark++;
            boolean ok = limelight.captureSnapshot(String.format(Locale.US, "mark_%03d", mark));
            logEvent("MARK", "snapshot=" + ok);
        }
        if (pressed(8, gamepad1.left_stick_button)) {
            runSwitchTest();
        }
        if (pressed(9, gamepad1.right_stick_button)) {
            clipping = !clipping;
            if (clipping) {
                clip++;
                clipFrame = 0;
            }
            logEvent(clipping ? "CLIP_START" : "CLIP_STOP", "clip=" + clip);
        }
    }

    /** Stores this mode's servo position (and, for ball mode, pitch) in PerceptionConfig and logs it. */
    private void saveSetpoint() {
        setpoint[mode.ordinal()] = servoPos;
        if (mode == Mode.BALL) {
            PerceptionConfig.TILT_BALL_POS = servoPos;
            PerceptionConfig.BALL_PITCH_DOWN_DEG = pitchDeg[Mode.BALL.ordinal()];
        } else {
            PerceptionConfig.TILT_GOAL_POS = servoPos;
        }
        logEvent("SAVE", String.format(Locale.US, "%s servo=%.3f pitch_deg=%.1f",
                mode, servoPos, pitchDeg[mode.ordinal()]));
    }

    private void setServo(double pos) {
        servoPos = Range.clip(pos, 0.0, 1.0);
        if (tilt != null) {
            tilt.setPosition(servoPos);
        }
    }

    private void enter(Mode m) {
        mode = m;
        limelight.pipelineSwitch(pipelineFor(m));
        setServo(setpoint[m.ordinal()]);
        logEvent("MODE", m.name());
    }

    private int pipelineFor(Mode m) {
        return m == Mode.BALL ? ballPipeline.index : PerceptionConfig.GOAL_PIPELINE;
    }

    /* ---------------- switch timing ---------------- */

    /** Times each switch from the command to (a) the first result from the new
     *  pipeline and (b) the first result with a valid target. (b) includes the
     *  servo's travel, since the target only enters the frame once it arrives.
     *  Use the medians to set PerceptionConfig.MODE_SETTLE_MS. */
    private void runSwitchTest() {
        long[] toPipe = new long[SWITCH_TRIALS];
        long[] toValid = new long[SWITCH_TRIALS];
        for (int i = 0; i < SWITCH_TRIALS && opModeIsActive(); i++) {
            Mode target = mode == Mode.BALL ? Mode.GOAL : Mode.BALL;
            long t0 = System.currentTimeMillis();
            enter(target);
            toPipe[i] = -1;
            toValid[i] = -1;
            while (opModeIsActive() && System.currentTimeMillis() - t0 < SWITCH_TIMEOUT_MS) {
                LLResult r = limelight.getLatestResult();
                if (r != null && r.getControlHubTimeStamp() > t0 && r.getPipelineIndex() == pipelineFor(target)) {
                    if (toPipe[i] < 0) {
                        toPipe[i] = r.getControlHubTimeStamp() - t0;
                    }
                    if (r.isValid()) {
                        toValid[i] = r.getControlHubTimeStamp() - t0;
                        break;
                    }
                }
                telemetry.addData("Switch test", "%d/%d -> %s", i + 1, SWITCH_TRIALS, target);
                telemetry.update();
            }
            logEvent("SWITCH", String.format(Locale.US, "to=%s pipeline_ms=%d valid_ms=%d",
                    target, toPipe[i], toValid[i]));
            sleep(SWITCH_PAUSE_MS);
        }
        switchSummary = String.format(Locale.US, "median pipeline %s ms, valid target %s ms (-1 = timed out)",
                median(toPipe), median(toValid));
        logEvent("SWITCH_SUMMARY", switchSummary);
    }

    private static String median(long[] v) {
        List<Long> ok = new ArrayList<Long>();
        for (long x : v) {
            if (x >= 0) {
                ok.add(x);
            }
        }
        Collections.sort(ok);
        int timeouts = v.length - ok.size();
        String m = ok.isEmpty() ? "-" : String.valueOf(ok.get(ok.size() / 2));
        return timeouts == 0 ? m : m + " (" + timeouts + " timed out)";
    }

    /* ---------------- snapshots (capture sessions) ---------------- */

    private void snapshotClipFrame() {
        long now = System.currentTimeMillis();
        if (!clipping || now - lastSnapMs < SNAPSHOT_PERIOD_MS) {
            return;
        }
        lastSnapMs = now;
        clipFrame++;
        String name = String.format(Locale.US, "clip%03d_%04d", clip, clipFrame);
        if (!limelight.captureSnapshot(name)) {
            logEvent("SNAPSHOT_FAILED", name);
        }
    }

    /* ---------------- readout ---------------- */

    private void countFps(long stamp) {
        recentStamps.add(stamp);
        while (!recentStamps.isEmpty() && stamp - recentStamps.get(0) > 1000) {
            recentStamps.remove(0);
        }
    }

    /** Ball class for a detector class id; colour-pipeline blobs (no class) take the pipeline's colour. */
    private BallClass ballClass(int classId) {
        BallClass c = BallClass.fromClassId(classId);
        if (c == null && ballPipeline.type == PipelineSpec.Type.COLOUR) {
            c = ballPipeline.colourClass;
        }
        return c == null ? BallClass.POLLEN : c;
    }

    /**
     * Floor position {forward, left} from the robot centre and the camera's horizontal range, or null above
     * the horizon. Same mount as CameraMount.fromConfig() (offsets, height, turret yaw, FOV), with only the
     * pitch swapped for the bench's assumed value, so the readout matches the perception code.
     */
    private double[] project(BallClass c, double txDeg, double tyDeg) {
        CameraMount mount = new CameraMount(PerceptionConfig.CAM_FORWARD_IN, PerceptionConfig.CAM_LEFT_IN,
                PerceptionConfig.CAM_HEIGHT_IN, pitchDeg[Mode.BALL.ordinal()], PerceptionConfig.TURRET_BALL_YAW_DEG,
                PerceptionConfig.HFOV_DEG, PerceptionConfig.VFOV_DEG);
        return BallProjector.project(new Pose(0, 0, 0), mount, txDeg, tyDeg, c.centerHeightIn());
    }

    private void show(LLResult r) {
        telemetry.addData("Mode", "%s  (A ball / Y goal)", mode);
        telemetry.addData("Servo", tilt == null ? "none (manual angle)" : String.format(Locale.US,
                "%.2f  setpoints ball %.2f goal %.2f  (X saves)", servoPos, setpoint[0], setpoint[1]));
        telemetry.addData("Assumed pitch", "%.1f deg below horizontal (D-pad L/R)", pitchDeg[mode.ordinal()]);
        telemetry.addData("Lens height", "%.1f in (PerceptionConfig.CAM_HEIGHT_IN)", PerceptionConfig.CAM_HEIGHT_IN);
        if (r == null) {
            telemetry.addLine("Waiting for a result from pipeline " + pipelineFor(mode) + ".");
        } else {
            telemetry.addData("Pipeline", "%d (%s)  stale %d ms  %d fps",
                    r.getPipelineIndex(), r.getPipelineType(), r.getStaleness(), recentStamps.size());
            if (mode == Mode.BALL) {
                showBalls(r);
            } else {
                showTags(r);
            }
        }
        telemetry.addData("Mark", "%d  (B = next mark + snapshot)", mark);
        telemetry.addData("Clip", clipping ? "RECORDING clip " + clip + ", frame " + clipFrame
                : "off  (right stick button)");
        telemetry.addData("Switch test", switchSummary);
        telemetry.addLine(csvStatus);
        telemetry.update();
    }

    private void showBalls(LLResult r) {
        int shown = 0;
        List<LLResultTypes.DetectorResult> dets = r.getDetectorResults();
        if (dets != null) {
            for (LLResultTypes.DetectorResult d : dets) {
                if (shown++ >= MAX_SHOWN) {
                    break;
                }
                showBall(d.getClassName(), ballClass(d.getClassId()), d.getConfidence(),
                        d.getTargetXDegrees(), d.getTargetYDegrees());
            }
        }
        List<LLResultTypes.ColorResult> blobs = r.getColorResults();
        if (blobs != null) {
            for (LLResultTypes.ColorResult b : blobs) {
                if (shown++ >= MAX_SHOWN) {
                    break;
                }
                showBall("blob", ballClass(-1), Double.NaN, b.getTargetXDegrees(), b.getTargetYDegrees());
            }
        }
        if (shown == 0) {
            telemetry.addLine("No balls detected.");
        }
    }

    private void showBall(String name, BallClass c, double conf, double tx, double ty) {
        double[] p = project(c, tx, ty);
        telemetry.addLine(String.format(Locale.US, "%-13s %.2f  tx %+5.1f ty %+5.1f  %s",
                name, conf, tx, ty,
                p == null ? "above horizon" : String.format(Locale.US, "range %.1f in (robot fwd %.1f, left %+.1f)",
                        p[2], p[0], p[1])));
    }

    private void showTags(LLResult r) {
        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addLine("No tags detected.");
            return;
        }
        for (int i = 0; i < Math.min(MAX_SHOWN, tags.size()); i++) {
            LLResultTypes.FiducialResult t = tags.get(i);
            // elevation above horizontal = ty + camera tilt up = ty - pitch below horizontal
            telemetry.addLine(String.format(Locale.US, "tag %d  tx %+5.1f ty %+5.1f  elevation %+.1f deg",
                    t.getFiducialId(), t.getTargetXDegrees(), t.getTargetYDegrees(),
                    t.getTargetYDegrees() - pitchDeg[Mode.GOAL.ordinal()]));
        }
    }

    /* ---------------- CSV ---------------- */

    private void openCsv() {
        File dir = new File(AppUtil.FIRST_FOLDER, "perception");
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());
        try {
            if (!dir.isDirectory() && !dir.mkdirs()) {
                throw new IOException("cannot create " + dir);
            }
            // createNewFile is atomic and never overwrites: a restart can't replace an earlier session's log.
            File f = new File(dir, "prelim_" + stamp + ".csv");
            for (int n = 2; !f.createNewFile(); n++) {
                f = new File(dir, "prelim_" + stamp + "_" + n + ".csv");
            }
            csv = new PrintWriter(new FileWriter(f, true));
            // dist_in = camera horizontal range; fwd_in/left_in = from the robot centre
            csv.println("ms,event,mode,servo,pitch_deg,pipeline,stale_ms,mark,kind,id,conf,tx,ty,dist_in,fwd_in,left_in,note");
            csvStatus = "Logging to " + f.getPath();
        } catch (IOException e) {
            csv = null;
            csvStatus = "CSV LOGGING OFF: " + e.getMessage();
        }
    }

    private void row(String event, int pipeline, long stale, String kind, String id, double conf,
                     double tx, double ty, double[] p, String note) {
        if (csv == null) {
            return;
        }
        csv.println(String.format(Locale.US, "%d,%s,%s,%.3f,%.1f,%d,%d,%d,%s,%s,%.3f,%.2f,%.2f,%s,%s,%s,%s",
                System.currentTimeMillis(), event, mode, servoPos, pitchDeg[mode.ordinal()], pipeline, stale,
                mark, quote(kind), quote(id), conf, tx, ty,
                p == null ? "" : String.format(Locale.US, "%.2f", p[2]),
                p == null ? "" : String.format(Locale.US, "%.2f", p[0]),
                p == null ? "" : String.format(Locale.US, "%.2f", p[1]),
                quote(note)));
        csv.flush();
    }

    /** RFC 4180 quoting for free-form fields: wrap in quotes when needed and double any quotes inside. */
    static String quote(String s) {
        if (s == null) {
            return "";
        }
        if (s.indexOf(',') < 0 && s.indexOf('"') < 0 && s.indexOf('\n') < 0 && s.indexOf('\r') < 0) {
            return s;
        }
        return "\"" + s.replace("\"", "\"\"") + "\"";
    }

    private void logEvent(String event, String note) {
        row(event, pipelineFor(mode), -1, "", "", Double.NaN, Double.NaN, Double.NaN, null, note);
    }

    /** One row per target in a fresh result, or one "none" row so misses are recorded too. */
    private void logResult(LLResult r) {
        int pipe = r.getPipelineIndex();
        long stale = r.getStaleness();
        boolean any = false;
        List<LLResultTypes.DetectorResult> dets = r.getDetectorResults();
        if (dets != null) {
            for (LLResultTypes.DetectorResult d : dets) {
                any = true;
                BallClass c = ballClass(d.getClassId());
                row("RESULT", pipe, stale, "ball", String.valueOf(d.getClassId()), d.getConfidence(),
                        d.getTargetXDegrees(), d.getTargetYDegrees(),
                        project(c, d.getTargetXDegrees(), d.getTargetYDegrees()), d.getClassName());
            }
        }
        List<LLResultTypes.ColorResult> blobs = r.getColorResults();
        if (blobs != null) {
            for (LLResultTypes.ColorResult b : blobs) {
                any = true;
                BallClass c = ballClass(-1);
                row("RESULT", pipe, stale, "blob", c.name(), Double.NaN,
                        b.getTargetXDegrees(), b.getTargetYDegrees(),
                        project(c, b.getTargetXDegrees(), b.getTargetYDegrees()), "");
            }
        }
        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags != null) {
            for (LLResultTypes.FiducialResult t : tags) {
                any = true;
                row("RESULT", pipe, stale, "tag", String.valueOf(t.getFiducialId()), Double.NaN,
                        t.getTargetXDegrees(), t.getTargetYDegrees(), null, t.getFamily());
            }
        }
        if (!any) {
            row("RESULT", pipe, stale, "none", "", Double.NaN, Double.NaN, Double.NaN, null, "");
        }
    }
}
