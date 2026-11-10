package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;

import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Everything strategic about one collect run. Defaults allow everything that is safe. */
public final class CollectRequest {
    public final Alliance alliance;
    public final boolean auto;
    public final Set<BallClass> classes;
    public final ZonePreset zone;
    public final Set<String> sources;
    public final boolean useHints;
    public final int maxTargets;          // <= 0 = intake capacity
    public final double timeoutSec;       // <= 0 = none
    public final boolean endWhenFull;
    private final Map<BallClass, Double> weights;

    private CollectRequest(Builder b) {
        this.alliance = b.alliance;
        this.auto = b.auto;
        this.classes = Collections.unmodifiableSet(EnumSet.copyOf(b.classes));
        this.zone = b.zone;
        this.sources = Collections.unmodifiableSet(new HashSet<String>(b.sources));
        this.useHints = b.useHints;
        this.maxTargets = b.maxTargets;
        this.timeoutSec = b.timeoutSec;
        this.endWhenFull = b.endWhenFull;
        this.weights = new EnumMap<BallClass, Double>(b.weights);
    }

    public double weight(BallClass c) {
        Double w = weights.get(c);
        return w == null ? 1.0 : w;
    }

    public static Builder builder(Alliance alliance, boolean auto) {
        return new Builder(alliance, auto);
    }

    public static final class Builder {
        private final Alliance alliance;
        private final boolean auto;
        private EnumSet<BallClass> classes;
        private final EnumMap<BallClass, Double> weights = new EnumMap<BallClass, Double>(BallClass.class);
        private ZonePreset zone = ZonePreset.WHOLE_FIELD;
        private final Set<String> sources = new HashSet<String>();
        private boolean useHints = true;
        private int maxTargets = -1;
        private double timeoutSec;
        private boolean endWhenFull = true;

        private Builder(Alliance alliance, boolean auto) {
            this.alliance = alliance;
            this.auto = auto;
            this.classes = EnumSet.of(BallClass.POLLEN);
            if (CollectConfig.COLLECT_NECTAR) {
                classes.add(alliance.ownNectar());
            }
            for (BallSource s : BallSource.ALL) {
                if (s.enabledByDefault && s.usableBy(alliance)) {
                    sources.add(s.name);
                }
            }
            this.timeoutSec = auto ? CollectConfig.AUTO_TIMEOUT_SEC : 0;
        }

        public Builder classes(BallClass... cs) {
            classes = EnumSet.noneOf(BallClass.class);
            Collections.addAll(classes, cs);
            return this;
        }

        public Builder weight(BallClass c, double w) {
            weights.put(c, w);
            return this;
        }

        public Builder zone(ZonePreset z) {
            zone = z;
            return this;
        }

        public Builder enableSource(String name) {
            sources.add(name);
            return this;
        }

        public Builder disableSource(String name) {
            sources.remove(name);
            return this;
        }

        public Builder onlySources(String... names) {
            sources.clear();
            Collections.addAll(sources, names);
            return this;
        }

        public Builder useHints(boolean v) {
            useHints = v;
            return this;
        }

        public Builder maxTargets(int n) {
            maxTargets = n;
            return this;
        }

        public Builder timeoutSec(double s) {
            timeoutSec = s;
            return this;
        }

        public Builder endWhenFull(boolean v) {
            endWhenFull = v;
            return this;
        }

        public CollectRequest build() {
            return new CollectRequest(this);
        }
    }
}
