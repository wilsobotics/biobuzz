package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The one place our code gets robot parts other people own. The drivetrain team replaces the
 * body with their Pedro 3 Follower (Mecanum + Pinpoint constants). Until then it fails loudly.
 */
public final class RobotHooks {
    private RobotHooks() {
    }

    public static Follower createFollower(HardwareMap hw) {
        throw new IllegalStateException(
                "RobotHooks.createFollower: plug in the drivetrain team's Pedro 3 Follower here");
    }
}
