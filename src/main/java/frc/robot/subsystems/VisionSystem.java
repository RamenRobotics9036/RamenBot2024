package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
    @SuppressWarnings("LineLengthCheck")
    private final double m_limelightMountAngleRadiansY = VisionConstants.limelightMountAngleRadiansY;
    private final double m_limelightMountAngleRadiansX = VisionConstants.limelightMountAngleRadiansX;

    private final double m_limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double m_aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private NetworkTable m_limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry m_tableX = m_limelightTable.getEntry("tx");
    private NetworkTableEntry m_tableY = m_limelightTable.getEntry("ty");
    private NetworkTableEntry m_tableArea = m_limelightTable.getEntry("ta");
    private NetworkTableEntry m_tableId = m_limelightTable.getEntry("tid");

    private final double m_epsilon = 0.0000001;

    public VisionSystem() {
        // displayToShuffleBoard();
    }

    @SuppressWarnings("unused")
    private void displayToShuffleBoard() {
        ShuffleboardLayout visionLayout = Shuffleboard.getTab("Vision").getLayout(
                "April Tags",
                BuiltInLayouts.kList);
        visionLayout.addDouble("Raw Y", () -> getY());
        visionLayout.addDouble("X Displacement", () -> getxRadians());
        visionLayout.addDouble("Y Displacement", () -> getyRadians());
        visionLayout.addDouble("Area", () -> getArea());
        visionLayout.addBoolean("Is Detecting", () -> isDetected());
        visionLayout.addDouble("Distance Meters X", () -> getDistanceMetersX());
        visionLayout.addDouble("Distance Meters Y", () -> getDistanceMetersY());

        visionLayout.addDouble(
                "X tangent",
                () -> Math.tan(getxRadians() + m_limelightMountAngleRadiansX));
        visionLayout.addDouble(
                "Y tangent",
                () -> Math.tan(getyRadians() + m_limelightMountAngleRadiansY));
        visionLayout.addDouble("ID", () -> getId());
    }

    /**
     * X angle, left-right, from April tag. X cross-hair angle.
     */
    public double getX() {
        return m_tableX.getDouble(0);
    }

    /**
     * Y angle, up-down, to April tag. Y cross-hair angle.
     */
    public double getY() {
        return m_tableY.getDouble(0);
    }

    public double getxRadians() {
        return Math.toRadians(getX());
    }

    public double getyRadians() {
        return Math.toRadians(getY());
    }

    /**
     * Area of April tag in view.
     */
    public double getArea() {
        return m_tableArea.getDouble(0);
    }

    public boolean isDetected() {
        return getX() + getY() + getArea() != 0;
    }

    public double getId() {
        return m_tableId.getDouble(0);
    }

    public boolean isDetectedIdValid() {
        double myId = getId();
        if (Constants.VisionConstants.targetedIDList.contains(myId)) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Distance to April tag in meters Y.
     */
    public double getDistanceMetersY() {
        double angleToGoalRadians = m_limelightMountAngleRadiansY + getyRadians();
        double distanceFromLimelightToGoalMeters = (m_aprilTagHeightMeters
                - m_limelightLensHeightMeters) / (Math.tan(angleToGoalRadians) + m_epsilon);
        return distanceFromLimelightToGoalMeters;
    }

    /**
     * Distance to April tag in meters X.
     */
    public double getDistanceMetersX() {
        double angleToGoalRadians = m_limelightMountAngleRadiansX + getxRadians();
        double distanceFromLimelightToGoalMeters = getDistanceMetersY()
                * Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
    }

    public void stopSystem() {
    }
}
