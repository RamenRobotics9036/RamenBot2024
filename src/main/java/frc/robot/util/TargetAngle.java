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
    public TargetAngle() {
        var lookUpVals = VisionConstants.angleLookUpTable;
        m_sortedAngleLookUpTable = new ArrayList<>(lookUpVals);
        m_sortedAngleLookUpTable.sort((a, b) -> a.getFirst().compareTo(b.getFirst()));
    }

    public double getShootingAngle(double distance) {
        if (distance < ArmConstants.lookUpTableDistance) {
            // This is our well-known scoring angle
            return PresetConstants.speakerPresetAngleRadians;
        }
        else {
            return 5;
        }
    }
}
