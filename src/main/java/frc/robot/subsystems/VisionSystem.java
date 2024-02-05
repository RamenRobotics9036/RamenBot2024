package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
    private final double limelightMountAngleRadiansY = VisionConstants.limelightMountAngleRadiansY;
    private final double limelightMountAngleRadiansX = VisionConstants.limelightMountAngleRadiansX;

    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.limelightName);
    private NetworkTableEntry tableX = limelightTable.getEntry("tx");
    private NetworkTableEntry tableY = limelightTable.getEntry("ty");
    private NetworkTableEntry tableArea = limelightTable.getEntry("ta");

    private final double EPSILON = 0.0000001;

    public VisionSystem() {
        displayToShuffleBoard();
    }

    private void displayToShuffleBoard() {
        ShuffleboardLayout visionLayout = Shuffleboard.getTab("Vision")
                .getLayout("April Tags", BuiltInLayouts.kList);
        visionLayout.addDouble("X Displacement", () -> getXRadians());
        visionLayout.addDouble("Y Displacement", () -> getYRadians());
        visionLayout.addDouble("Area", () -> getArea());
        visionLayout.addBoolean("Is Detecting", () -> isDetected());
        visionLayout.addDouble("Distance Meters X", () -> getDistanceMetersX());
        visionLayout.addDouble("Distance Meters Y", () -> getDistanceMetersY());

        visionLayout.addDouble("X tangent", () -> Math.tan(getXRadians()));
        visionLayout.addDouble("Y tangent", () -> Math.tan(getYRadians()));
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

    public double getXRadians() {
        return Math.toRadians(getX());
    }

    public double getYRadians() {
        return Math.toRadians(getY());
    }

    /**
     * Area of April tag in view.
     */
    public double getArea() {
        return tableArea.getDouble(0);
    }

    public boolean isDetected() {
        return getX() + getY() + getArea() != 0;
    }

    /**
     * Distance to April tag in meters Y.
     */
    public double getDistanceMetersY() {
        double angleToGoalRadians = limelightMountAngleRadiansY + getYRadians();
        double distanceFromLimelightToGoalMeters = (aprilTagHeightMeters - limelightLensHeightMeters) / (Math.tan(angleToGoalRadians) + EPSILON);
        return distanceFromLimelightToGoalMeters;
    }

    /**
     * Distance to April tag in meters X.
     */
    public double getDistanceMetersX() {
        double angleToGoalRadians = limelightMountAngleRadiansX + getXRadians();
        double distanceFromLimelightToGoalMeters = getDistanceMetersY() * Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
    }

    public void stopSystem() { 
    }
}
