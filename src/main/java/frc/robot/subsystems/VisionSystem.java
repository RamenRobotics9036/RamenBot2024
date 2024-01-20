package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
    private final double limelightMountAngleRadians = VisionConstants.limelightMountAngleRadians;
    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry tableX = limelightTable.getEntry("tx");
    private NetworkTableEntry tableY = limelightTable.getEntry("ty");
    private NetworkTableEntry tableArea = limelightTable.getEntry("ta");

    public VisionSystem() {
        displayToShuffleBoard();
    }

    private void displayToShuffleBoard() {
        ShuffleboardLayout visionLayout = Shuffleboard.getTab("Vision").getLayout("April Tags",
                BuiltInLayouts.kList);
        visionLayout.addDouble("X Displacement", () -> getX());
        visionLayout.addDouble("Y Displacement", () -> getY());
        visionLayout.addDouble("Area", () -> getArea());
        visionLayout.addDouble("Distance Meters", () -> getDistanceMetersToGoal());
    }

    public double getXPose() {
        return LimelightHelpers.getBotPose2d(VisionConstants.limelightName).getX();
    }

    public double getYPose() {
        return LimelightHelpers.getBotPose2d(VisionConstants.limelightName).getY();
    }

    /**
     * X angle, left-right, from April tag. X cross-hair angle.
     */
    public double getX() {
        return tableX.getDouble(0);
    }

    /**
     * Y angle, up-down, to April tag. Y cross-hair angle.
     */
    public double getY() {
        return tableY.getDouble(0);
    }

    /**
     * Area of April tag in view.
     */
    public double getArea() {
        return tableArea.getDouble(0);
    }

    /**
     * Distance to April tag in meters.
     */
    public double getDistanceMetersToGoal() {
        double angleToGoalRadians = limelightMountAngleRadians + getY();
        double distanceFromLimelightToGoalMeters = (aprilTagHeightMeters
                - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
    }

    public void stopSystem() {
    }
}
