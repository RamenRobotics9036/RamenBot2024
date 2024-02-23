package frc.robot.util;

import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.VisionConstants;

public class AngleLookUpTable {
    private List<Pair<Double, Double>> m_distanceToAngleArray;

    public AngleLookUpTable() {
        m_distanceToAngleArray = VisionConstants.sortedAngleLookUpTable;
        m_distanceToAngleArray.sort(new ComparePair());
    }

    private class ComparePair implements Comparator<Pair<Double, Double>> {
        public int compare(Pair<Double, Double> a, Pair<Double, Double> b) {
            if (a.getFirst() >= b.getSecond()) {
                return 0;
            }
            return 1;
        }
    }

    public double getAngleAt(double distance) {
        return 0;
    }
}
