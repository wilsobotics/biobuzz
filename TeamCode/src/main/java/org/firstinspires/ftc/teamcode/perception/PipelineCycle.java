package org.firstinspires.ftc.teamcode.perception;

import java.util.ArrayList;
import java.util.List;

/** The ball pipelines, visited round-robin when there is more than one. */
public final class PipelineCycle {
    private final List<PipelineSpec> specs;
    private int current = 0;

    public PipelineCycle(List<PipelineSpec> specs) {
        if (specs == null || specs.isEmpty()) {
            throw new IllegalArgumentException("At least one ball pipeline is required");
        }
        this.specs = new ArrayList<PipelineSpec>(specs);
    }

    /** Parses "1:NEURAL" or "1:COLOUR:POLLEN,2:COLOUR:RED_NECTAR,...". */
    public static PipelineCycle parse(String text) {
        ArrayList<PipelineSpec> out = new ArrayList<PipelineSpec>();
        for (String part : text.split(",")) {
            String[] f = part.trim().split(":");
            if (f.length == 2 && f[1].trim().equals("NEURAL")) {
                out.add(PipelineSpec.neural(Integer.parseInt(f[0].trim())));
            } else if (f.length == 3 && f[1].trim().equals("COLOUR")) {
                out.add(PipelineSpec.colour(Integer.parseInt(f[0].trim()), BallClass.valueOf(f[2].trim())));
            } else {
                throw new IllegalArgumentException("Bad ball pipeline entry: " + part);
            }
        }
        return new PipelineCycle(out);
    }

    public PipelineSpec current() {
        return specs.get(current);
    }

    public PipelineSpec advance() {
        current = (current + 1) % specs.size();
        return current();
    }

    public int size() {
        return specs.size();
    }
}
