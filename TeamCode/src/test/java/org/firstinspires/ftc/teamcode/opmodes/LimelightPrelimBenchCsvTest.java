package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LimelightPrelimBenchCsvTest {
    @Test
    public void plainFieldsAreLeftAlone() {
        assertEquals("yellow_pollen", LimelightPrelimBench.quote("yellow_pollen"));
        assertEquals("", LimelightPrelimBench.quote(null));
    }

    @Test
    public void commasAndQuotesAreEscaped() {
        assertEquals("\"median pipeline 120 ms, valid target 180 ms\"",
                LimelightPrelimBench.quote("median pipeline 120 ms, valid target 180 ms"));
        assertEquals("\"say \"\"hi\"\"\"", LimelightPrelimBench.quote("say \"hi\""));
    }
}
