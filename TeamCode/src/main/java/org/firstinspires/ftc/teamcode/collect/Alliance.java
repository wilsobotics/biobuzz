package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;

public enum Alliance {
    RED,
    BLUE;

    public BallClass ownNectar() {
        return this == RED ? BallClass.RED_NECTAR : BallClass.BLUE_NECTAR;
    }
}
