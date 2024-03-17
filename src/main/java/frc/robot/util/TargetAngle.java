package frc.robot.util;

import java.util.Map;
import java.util.HashMap;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Figures out the angle to the target.
 */
public class TargetAngle {
    private final Map<Double, Double> lookUpTable = new HashMap<>();

    // Constructor
    public TargetAngle() {
        var lookUpVals = VisionConstants.sortedAngleLookUpTable;
        for (int idx = 0; idx < lookUpVals.size(); idx++) {
            lookUpTable.put(lookUpVals.get(idx).getFirst(), lookUpVals.get(idx).getSecond());
        }
    }

    public double getShootingAngle(double distance) {
        if (distance < ArmConstants.lookUpTableDistance) {
            return lookUpTable.get(0.);
        }
        try {
            double distanceCeil = Math.ceil(distance / ArmConstants.lookUpTableDistance)
                    * ArmConstants.lookUpTableDistance;
            double distanceFloor = Math.floor(distance / ArmConstants.lookUpTableDistance)
                    * ArmConstants.lookUpTableDistance;
            double angleCeil = lookUpTable.get(distanceCeil);
            double angleFloor = lookUpTable.get(distanceFloor);
            double percentDiv = distanceCeil - distanceFloor;

            return angleCeil * ((distanceCeil - distance) / percentDiv)
                    + angleFloor * ((distance - distanceFloor) / percentDiv);
        }
        catch (Exception e) {
            return lookUpTable.get(ArmConstants.lookUpTableDistance * 6);
        }
    }
}
