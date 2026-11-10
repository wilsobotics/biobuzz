package org.firstinspires.ftc.teamcode.collect;

import org.firstinspires.ftc.teamcode.perception.BallClass;
import org.firstinspires.ftc.teamcode.perception.BallMap;
import org.firstinspires.ftc.teamcode.perception.Sighting;

import java.util.ArrayList;

/** Seeds confirmed balls: confirmed(map, BallClass.POLLEN, 40.0, 20.0, BallClass.RED_NECTAR, 60.0, 30.0, ...). */
final class MapSeeds {
    private MapSeeds() {
    }

    static void confirmed(BallMap map, Object... classXY) {
        seed(map, 1.0, 0.9, false, classXY);
    }

    static void hints(BallMap map, Object... classXY) {
        seed(map, 6.0, 0.25, false, classXY);
    }

    static void maybeTwo(BallMap map, Object... classXY) {
        seed(map, 1.0, 0.9, true, classXY);
    }

    private static void seed(BallMap map, double sigma, double q, boolean two, Object... classXY) {
        ArrayList<Sighting> ss = new ArrayList<Sighting>();
        for (int i = 0; i < classXY.length; i += 3) {
            ss.add(new Sighting((BallClass) classXY[i], (Double) classXY[i + 1], (Double) classXY[i + 2],
                    30, sigma, q, two));
        }
        map.update(0, null, null, ss);
        map.update(10, null, null, ss);
    }
}
