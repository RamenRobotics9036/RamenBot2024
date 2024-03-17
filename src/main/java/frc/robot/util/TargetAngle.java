package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.Pair;

import java.util.HashMap;
import java.util.ArrayList;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Figures out the angle to the target.
 */
public class TargetAngle {
    private final ArrayList<Pair<Double, Double>> m_sortedAngleLookUpTable;

    // Constructor
    public TargetAngle(ArrayList<Pair<Double, Double>> lookUpVals) {
        m_sortedAngleLookUpTable = new ArrayList<>(lookUpVals);
        m_sortedAngleLookUpTable.sort((a, b) -> a.getFirst().compareTo(b.getFirst()));
    }

    public double getShootingAngle(double distance) {
        if (distance < ArmConstants.lookUpTableDistance) {
            // This is our well-known scoring angle
            return PresetConstants.speakerPresetAngleRadians;
        }
        else {
            return getAngleHelper(distance);
        }
    }

    private double getAngleHelper(double distance) {
        // Check for edge cases: distance smaller than the first entry or larger than the last entry
        if (m_sortedAngleLookUpTable.isEmpty() || distance < 0) {
            return 0;
        }

        int lastIndex = m_sortedAngleLookUpTable.size() - 1;

        // If we're before the first item of the table, we interpolate
        if (distance < m_sortedAngleLookUpTable.get(0).getFirst()) {
            double lastDistance = m_sortedAngleLookUpTable.get(0).getFirst();
            double lastAngle = m_sortedAngleLookUpTable.get(0).getSecond();
            double currentDistance = m_sortedAngleLookUpTable.get(1).getFirst();
            double currentAngle = m_sortedAngleLookUpTable.get(1).getSecond();

            double ratio = (distance - lastDistance) / (currentDistance - lastDistance);
            double angleDifference = currentAngle - lastAngle;
            return lastAngle + ratio * angleDifference;
        }

        // If we're past the end of the table, we interpolate based on the last two entries
        if (distance > m_sortedAngleLookUpTable.get(lastIndex).getFirst()) {
            double lastDistance = m_sortedAngleLookUpTable.get(lastIndex - 1).getFirst();
            double lastAngle = m_sortedAngleLookUpTable.get(lastIndex - 1).getSecond();
            double currentDistance = m_sortedAngleLookUpTable.get(lastIndex).getFirst();
            double currentAngle = m_sortedAngleLookUpTable.get(lastIndex).getSecond();

            double ratio = (distance - lastDistance) / (currentDistance - lastDistance);
            double angleDifference = currentAngle - lastAngle;
            return lastAngle + ratio * angleDifference;
        }

        // Linear search to find the correct interval for interpolation
        // For a large table, consider using a more efficient search method like binary search
        for (int i = 0; i < m_sortedAngleLookUpTable.size() - 1; i++) {
            Pair<Double, Double> current = m_sortedAngleLookUpTable.get(i);
            Pair<Double, Double> next = m_sortedAngleLookUpTable.get(i + 1);

            if (distance >= current.getFirst() && distance <= next.getFirst()) {
                // Perform linear interpolation
                double ratio = (distance - current.getFirst())
                        / (next.getFirst() - current.getFirst());
                double angleDifference = next.getSecond() - current.getSecond();
                return current.getSecond() + ratio * angleDifference;
            }
        }

        // Default return value in case no suitable interval was found, should never reach here if
        // input is validated
        return 0;
    }
}
