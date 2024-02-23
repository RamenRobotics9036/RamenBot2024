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
        Double a = null;
        Double b = null;
        Double aError = Double.MAX_VALUE;
        Double bError = Double.MAX_VALUE;

        Double aVal = 0d;
        Double bVal = 0d;

        for (int idx = 0; idx < m_distanceToAngleArray.size(); idx++) {
            if (a == null) {
                a = m_distanceToAngleArray.get(idx).getFirst();
                aError = Math.abs(m_distanceToAngleArray.get(idx).getFirst() - distance);
                aVal = m_distanceToAngleArray.get(idx).getSecond();
            }
            else if (b == null) {
                b = m_distanceToAngleArray.get(idx).getFirst();
                bError = Math.abs(m_distanceToAngleArray.get(idx).getFirst() - distance);
                bVal = m_distanceToAngleArray.get(idx).getSecond();
            }
            if (Math.abs(m_distanceToAngleArray.get(idx).getFirst() - distance) < aError) {
                aError = m_distanceToAngleArray.get(idx).getFirst() - distance;
                a = m_distanceToAngleArray.get(idx).getFirst();
                aVal = m_distanceToAngleArray.get(idx).getSecond();

                bError = aError.doubleValue();
                b = a.doubleValue();
                bVal = aVal.doubleValue();
            }
            else if (Math.abs(m_distanceToAngleArray.get(idx).getFirst() - distance) < bError) {
                bError = m_distanceToAngleArray.get(idx).getFirst() - distance;
                b = m_distanceToAngleArray.get(idx).getFirst();
                bVal = m_distanceToAngleArray.get(idx).getSecond();
            }
        }

        double aWeight = (a - b) / Math.abs((a - distance));
        double bWeight = (a - b) / Math.abs((b - distance));
        return (aVal * aWeight + bVal * bWeight) / 2;
    }
}
