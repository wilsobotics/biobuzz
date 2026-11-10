package org.firstinspires.ftc.teamcode.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

public class FrameGateTest {
    private PoseHistory poses;
    private FrameGate gate;
    private static final PipelineSpec NEURAL = PipelineSpec.neural(1);
    /** Turret ready long before any capture in these tests. */
    private static final long READY = 0;

    @Before
    public void setUp() {
        poses = new PoseHistory(5000);
        poses.add(0, new Pose(0, 0, 0));
        poses.add(1000, new Pose(100, 0, 0));
        gate = new FrameGate();
        gate.expect(1, 0, 0);
    }

    private static RawResult raw(long ts, int pipeline, long stale, List<Detection> det, List<Detection> col) {
        return new RawResult(ts, stale, pipeline, 20, 30, det, col);
    }

    private static List<Detection> one(int classId) {
        return Collections.singletonList(new Detection(classId, 0.9, 1, 2, 1));
    }

    @Test
    public void acceptsAndStampsThePoseAtCaptureTime() {
        CameraFrame f = gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, READY);
        assertNotNull(f);
        assertEquals(1, f.id);
        assertEquals(450, f.captureTimeMs);
        assertEquals(45, f.robotPose.x(), 1e-9);
        assertEquals(1, f.balls.size());
        assertEquals(FrameGate.Reject.NONE, gate.lastReject());
    }

    @Test
    public void sameTimestampIsNotNew() {
        RawResult r = raw(500, 1, 10, one(0), Collections.<Detection>emptyList());
        assertNotNull(gate.accept(r, 520, CameraMode.BALL, NEURAL, poses, READY));
        assertNull(gate.accept(r, 530, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.NOT_NEW, gate.lastReject());
    }

    @Test
    public void wrongPipelineIsRejected() {
        assertNull(gate.accept(raw(500, 0, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.WRONG_PIPELINE, gate.lastReject());
    }

    @Test
    public void framesWaitForTheSettleTime() {
        gate.expect(1, 400, 250);
        assertNull(gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                500, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.SETTLING, gate.lastReject());
        assertNotNull(gate.accept(raw(700, 1, 10, one(0), Collections.<Detection>emptyList()),
                700, CameraMode.BALL, NEURAL, poses, READY));
    }

    @Test
    public void staleResultsAreRejected() {
        assertNull(gate.accept(raw(500, 1, 200, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.STALE, gate.lastReject());
    }

    @Test
    public void turretOffAngleRejectsBallFramesOnly() {
        assertNull(gate.accept(raw(500, 1, 10, one(0), Collections.<Detection>emptyList()),
                520, CameraMode.BALL, NEURAL, poses, FrameGate.TURRET_NOT_READY));
        assertEquals(FrameGate.Reject.TURRET, gate.lastReject());
        gate.expect(0, 0, 0);
        CameraFrame goal = gate.accept(raw(600, 0, 10, one(0), Collections.<Detection>emptyList()),
                620, CameraMode.GOAL, null, poses, FrameGate.TURRET_NOT_READY);
        assertNotNull(goal);
        assertEquals(0, goal.balls.size());
    }

    @Test
    public void frameCapturedBeforeTheTurretWasReadyIsRejected() {
        // ready since 600; raw(600) -> captured at 550
        assertNull(gate.accept(raw(600, 1, 10, one(0), Collections.<Detection>emptyList()),
                620, CameraMode.BALL, NEURAL, poses, 600));
        assertEquals(FrameGate.Reject.TURRET, gate.lastReject());
    }

    @Test
    public void frameCapturedAfterTheTurretWasReadyIsAccepted() {
        // ready since 600; raw(700) -> captured at 650
        CameraFrame f = gate.accept(raw(700, 1, 10, one(0), Collections.<Detection>emptyList()),
                720, CameraMode.BALL, NEURAL, poses, 600);
        assertNotNull(f);
        assertEquals(650, f.captureTimeMs);
    }

    @Test
    public void goalModeIgnoresTurretReadiness() {
        gate.expect(0, 0, 0);
        assertNotNull(gate.accept(raw(600, 0, 10, one(0), Collections.<Detection>emptyList()),
                620, CameraMode.GOAL, null, poses, 600));
    }

    @Test
    public void captureAfterTheNewestPoseWaitsForABracketingPose() {
        // newest pose at 1000; raw(1100) -> captured at 1050
        RawResult r = raw(1100, 1, 10, one(0), Collections.<Detection>emptyList());
        assertNull(gate.accept(r, 1110, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.POSE_PENDING, gate.lastReject());
        poses.add(1100, new Pose(200, 0, 0));
        CameraFrame f = gate.accept(r, 1120, CameraMode.BALL, NEURAL, poses, READY);
        assertNotNull(f);
        assertEquals(1050, f.captureTimeMs);
        assertEquals(150, f.robotPose.x(), 1e-9);
        assertNull(gate.accept(r, 1130, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.NOT_NEW, gate.lastReject());
    }

    @Test
    public void pendingFrameIsAcceptedWithTheNewestPoseAfterTheMaxWait() {
        RawResult r = raw(1100, 1, 10, one(0), Collections.<Detection>emptyList());
        assertNull(gate.accept(r, 1110, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.POSE_PENDING, gate.lastReject());
        assertNull(gate.accept(r, 1110 + PerceptionConfig.POSE_WAIT_MAX_MS - 1, CameraMode.BALL, NEURAL,
                poses, READY));
        assertEquals(FrameGate.Reject.POSE_PENDING, gate.lastReject());
        CameraFrame f = gate.accept(r, 1110 + PerceptionConfig.POSE_WAIT_MAX_MS, CameraMode.BALL, NEURAL,
                poses, READY);
        assertNotNull(f);
        assertEquals(100, f.robotPose.x(), 1e-9);
        assertEquals(FrameGate.Reject.NONE, gate.lastReject());
    }

    @Test
    public void colourPipelineAssignsItsClass() {
        gate.expect(2, 0, 0);
        CameraFrame f = gate.accept(raw(500, 2, 10, Collections.<Detection>emptyList(), one(-1)),
                520, CameraMode.BALL, PipelineSpec.colour(2, BallClass.RED_NECTAR), poses, READY);
        assertEquals(1, f.balls.get(0).classId);
    }

    @Test
    public void emptyFramesStillCount() {
        CameraFrame f = gate.accept(raw(500, 1, 10, Collections.<Detection>emptyList(),
                Collections.<Detection>emptyList()), 520, CameraMode.BALL, NEURAL, poses, READY);
        assertNotNull(f);
        assertEquals(0, f.balls.size());
    }

    @Test
    public void pipelineCycleParsesAndRotates() {
        assertEquals(1, PipelineCycle.parse("1:NEURAL").size());
        PipelineCycle c = PipelineCycle.parse("1:COLOUR:POLLEN, 2:COLOUR:RED_NECTAR");
        assertEquals(1, c.current().index);
        assertEquals(BallClass.POLLEN, c.current().colourClass);
        assertEquals(2, c.advance().index);
        assertEquals(1, c.advance().index);
    }

    @Test(expected = IllegalArgumentException.class)
    public void badPipelineTextIsRejected() {
        PipelineCycle.parse("1:FISH");
    }
    @Test
    public void settleIsJudgedByCaptureTimeNotPollTime() {
        gate.expect(1, 400, 250); // settled from 650
        // polled at 700 (after settle) but captured at 640 (before it)
        assertNull(gate.accept(raw(690, 1, 10, one(0), Collections.<Detection>emptyList()),
                700, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.SETTLING, gate.lastReject());
    }

    @Test
    public void repeatPollKeepsTheRejectReason() {
        gate.expect(1, 400, 250);
        RawResult r = raw(500, 1, 10, one(0), Collections.<Detection>emptyList());
        assertNull(gate.accept(r, 500, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.SETTLING, gate.lastReject());
        assertNull(gate.accept(r, 520, CameraMode.BALL, NEURAL, poses, READY));
        assertEquals(FrameGate.Reject.SETTLING, gate.lastReject());
    }
}
