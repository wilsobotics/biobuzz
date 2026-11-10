package org.firstinspires.ftc.teamcode.perception;

import com.pedropathing.math.Pose;

import java.util.ArrayList;

/** Recent robot poses, so a camera frame can be paired with the pose at its capture time. */
public final class PoseHistory {
    private static final class Entry {
        final long t;
        final Pose pose;

        Entry(long t, Pose pose) {
            this.t = t;
            this.pose = pose;
        }
    }

    private final long windowMs;
    private final ArrayList<Entry> entries = new ArrayList<Entry>();

    public PoseHistory(long windowMs) {
        this.windowMs = windowMs;
    }

    /** Adds a sample. Samples must arrive in time order; out-of-order or null samples are ignored. */
    public synchronized void add(long tMs, Pose pose) {
        if (pose == null) {
            return;
        }
        if (!entries.isEmpty() && tMs <= entries.get(entries.size() - 1).t) {
            return;
        }
        entries.add(new Entry(tMs, pose));
        long cutoff = tMs - windowMs;
        while (entries.size() > 2 && entries.get(1).t <= cutoff) {
            entries.remove(0);
        }
    }

    /** Pose at tMs: interpolated inside the history, newest after it, null before it. */
    public synchronized Pose at(long tMs) {
        if (entries.isEmpty()) {
            return null;
        }
        Entry last = entries.get(entries.size() - 1);
        if (tMs >= last.t) {
            return last.pose;
        }
        if (tMs < entries.get(0).t) {
            return null;
        }
        for (int i = entries.size() - 1; i > 0; i--) {
            Entry a = entries.get(i - 1);
            Entry b = entries.get(i);
            if (tMs >= a.t) {
                double f = (double) (tMs - a.t) / (double) (b.t - a.t);
                return lerp(a.pose, b.pose, f);
            }
        }
        return entries.get(0).pose;
    }

    static Pose lerp(Pose a, Pose b, double f) {
        double dh = Angles.wrap(b.heading() - a.heading());
        return new Pose(a.x() + (b.x() - a.x()) * f,
                a.y() + (b.y() - a.y()) * f,
                Angles.wrap(a.heading() + dh * f));
    }
}
