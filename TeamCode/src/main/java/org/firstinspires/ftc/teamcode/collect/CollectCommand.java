package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallTrackingSubsystem;
import org.firstinspires.ftc.teamcode.perception.CameraMode;
import org.firstinspires.ftc.teamcode.perception.CameraSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

/**
 * NextFTC wrapper around CollectBrain. One instance = one run: build a new one each time.
 * Ends with result(): FULL, NO_TARGETS, TIMEOUT or INTERRUPTED. Chaining SCORE is the caller's job.
 */
public final class CollectCommand extends Command {
    private final CameraSubsystem camera;
    private final CollectBrain brain;

    public CollectCommand(CameraSubsystem camera, BallTrackingSubsystem tracking, PathFollower follower,
                          IntakeStatus intake, CollectRequest request, List<ObstacleProvider> providers) {
        this.camera = camera;
        this.brain = new CollectBrain(request, tracking.map(), follower, intake, FieldFrame.fromConfig(), providers);
        named("Collect");
    }

    /**
     * As above, plus NextFTC requirements. Give collect a requirement it shares with the TeleOp
     * drive command (e.g. the drivetrain/follower subsystem) so the two never drive at once.
     */
    public CollectCommand(CameraSubsystem camera, BallTrackingSubsystem tracking, PathFollower follower,
                          IntakeStatus intake, CollectRequest request, List<ObstacleProvider> providers,
                          Object... requirements) {
        this(camera, tracking, follower, intake, request, providers);
        requires(requirements);
    }

    @Override
    public void start() {
        camera.setMode(CameraMode.BALL);
        brain.start(System.currentTimeMillis());
    }

    @Override
    public void update() {
        brain.update(System.currentTimeMillis());
    }

    @Override
    public boolean isDone() {
        return brain.isDone();
    }

    @Override
    public void stop(boolean interrupted) {
        brain.stop();
    }

    /** TeleOp: runs while the button is held; releasing it or moving a stick cancels at once. */
    public Command asTeleopAssist(final BooleanSupplier held, final DoubleSupplier stick) {
        return until(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return !held.getAsBoolean() || Math.abs(stick.getAsDouble()) > CollectConfig.STICK_CANCEL;
            }
        });
    }

    public CollectBrain brain() { return brain; }
    public CollectResult result() { return brain.result(); }
}
