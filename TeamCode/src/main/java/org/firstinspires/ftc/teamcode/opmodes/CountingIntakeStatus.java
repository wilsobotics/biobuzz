package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.collect.IntakeStatus;

/** Bench stand-in for the intake: a driver presses a button when a ball goes in. */
public final class CountingIntakeStatus implements IntakeStatus {
    private final int capacity;
    private int count;

    public CountingIntakeStatus(int capacity) {
        this.capacity = capacity;
    }

    public void increment() { count = Math.min(capacity, count + 1); }
    public void reset() { count = 0; }
    @Override public int ballCount() { return count; }
    @Override public int capacity() { return capacity; }
}
