package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.collect.Alliance;
import org.firstinspires.ftc.teamcode.collect.CollectCommand;
import org.firstinspires.ftc.teamcode.collect.CollectConfig;
import org.firstinspires.ftc.teamcode.collect.CollectRequest;
import org.firstinspires.ftc.teamcode.collect.CollectResult;
import org.firstinspires.ftc.teamcode.collect.FieldFrame;
import org.firstinspires.ftc.teamcode.collect.ObstacleProvider;
import org.firstinspires.ftc.teamcode.collect.PedroPathFollower;
import org.firstinspires.ftc.teamcode.collect.ZonePreset;
import org.firstinspires.ftc.teamcode.debug.FieldDrawer;
import org.firstinspires.ftc.teamcode.perception.BallTrackingSubsystem;
import org.firstinspires.ftc.teamcode.perception.CameraSubsystem;
import org.firstinspires.ftc.teamcode.perception.PoseSource;

import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Bench/practice OpMode. Init: D-pad up = alliance, D-pad right = zone preset.
 * Run: press right bumper = collect (release or move a stick to cancel), X = "ball went in",
 * Y = reset count; otherwise sticks drive through the follower.
 */
@TeleOp(name = "Collect Test", group = "Perception")
public final class CollectTest extends NextFTCOpMode {
    private final CameraSubsystem camera = new CameraSubsystem();
    private final BallTrackingSubsystem tracking = new BallTrackingSubsystem(camera);
    private final CountingIntakeStatus intake = new CountingIntakeStatus(CollectConfig.INTAKE_CAPACITY);
    private Follower follower;
    private PedroPathFollower pathFollower;
    private Alliance alliance = Alliance.RED;
    private ZonePreset zone = ZonePreset.WHOLE_FIELD;
    private CollectCommand active;
    private Command activeAssist;
    private boolean lastUp;
    private boolean lastRight;
    private boolean lastX;
    private boolean lastY;
    private boolean lastBumper;
    private CollectResult lastResult;

    public CollectTest() {
        addComponents(new SubsystemComponent(camera, tracking));
    }

    @Override
    public void onInit() {
        follower = RobotHooks.createFollower(hardwareMap);
        pathFollower = new PedroPathFollower(follower);
        camera.setPoseSource(new PoseSource() {
            @Override
            public Pose pose() {
                return follower.pose();
            }
        });
    }

    @Override
    public void onWaitForStart() {
        follower.update();
        camera.recordPose(System.currentTimeMillis(), follower.pose());
        if (gamepad1.dpad_up && !lastUp) {
            alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }
        if (gamepad1.dpad_right && !lastRight) {
            zone = zone.next();
        }
        lastUp = gamepad1.dpad_up;
        lastRight = gamepad1.dpad_right;
        telemetry.addData("Alliance (dpad up)", alliance);
        telemetry.addData("Zone (dpad right)", zone);
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        follower.update();
        camera.recordPose(System.currentTimeMillis(), follower.pose());
        if (gamepad1.x && !lastX) {
            intake.increment();
        }
        if (gamepad1.y && !lastY) {
            intake.reset();
        }
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (activeAssist != null && !activeAssist.isScheduled()) {
            lastResult = active.result();
            activeAssist = null;
        }
        boolean bumperPressed = gamepad1.right_bumper && !lastBumper;
        lastBumper = gamepad1.right_bumper;
        if (activeAssist == null && bumperPressed && stick() < CollectConfig.STICK_CANCEL) {
            active = new CollectCommand(camera, tracking, pathFollower, intake,
                    CollectRequest.builder(alliance, false).zone(zone).build(),
                    Collections.<ObstacleProvider>emptyList());
            activeAssist = active.asTeleopAssist(new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    return gamepad1.right_bumper;
                }
            }, new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return stick();
                }
            });
            activeAssist.schedule();
        }
        if (activeAssist == null) {
            // Robot-centric manual drive; check the signs on the first run.
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        TelemetryPacket packet = new TelemetryPacket();
        FieldDrawer.draw(packet.fieldOverlay(), follower.pose(), tracking.map(),
                activeAssist != null ? active.brain() : null, FieldFrame.fromConfig());
        packet.put("balls", tracking.map().size());
        packet.put("targets", tracking.map().targets().size());
        packet.put("frame reject", String.valueOf(camera.lastReject()));
        packet.put("intake", intake.ballCount());
        if (activeAssist != null) {
            packet.put("collect state", active.brain().state());
            packet.put("last replan", String.valueOf(active.brain().lastReason()));
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("Balls", tracking.map().size());
        telemetry.addData("Intake", intake.ballCount());
        telemetry.addData("Last collect", lastResult);
        telemetry.update();
    }

    private double stick() {
        return Math.max(Math.abs(gamepad1.left_stick_x),
                Math.max(Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.right_stick_x)));
    }
}
