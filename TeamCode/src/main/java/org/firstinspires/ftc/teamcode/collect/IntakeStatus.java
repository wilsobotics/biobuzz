package org.firstinspires.ftc.teamcode.collect;

/** What the collect command needs from the intake. The intake owner writes an adapter. */
public interface IntakeStatus {
    int ballCount();

    int capacity();

    default boolean isFull() {
        return ballCount() >= capacity();
    }
}
