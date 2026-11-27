package org.firstinspires.ftc.teamcode;

/**
 * The alliance a robot is playing for.
 *
 * This used to be nested inside TipDetects, where it only picked an AprilTag ID range. The field
 * frame needs the same value (to mirror start poses and field landmarks, and to pick a FLOWER
 * side), so it lives here and both use it.
 */
public enum Alliance {
    RED,
    BLUE
}
