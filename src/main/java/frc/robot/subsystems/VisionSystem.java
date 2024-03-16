package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers;

import java.util.Map;

public class VisionSystem extends SubsystemBase {
    private final double limelightMountAngleRadiansY = VisionConstants.limelightMountAngleRadiansY;
    private final double limelightMountAngleRadiansX = VisionConstants.limelightMountAngleRadiansX;

    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry tableX = limelightTable.getEntry("tx");
    private NetworkTableEntry tableY = limelightTable.getEntry("ty");
    private NetworkTableEntry tableArea = limelightTable.getEntry("ta");
    private NetworkTableEntry tableID = limelightTable.getEntry("tid");

    private final Field2d m_fieldSim = new Field2d();

    private final double EPSILON = 0.0000001;

    public VisionSystem() {
        displayToShuffleBoard();
    }

    // $IDO - This is where the vision shuffleboard is done
    private void displayToShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        // $IDO - Good doc on limelight fields
        // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
        // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
        // Super simple example:
        // https://github.com/Lambda-Corps/2020InfiniteRecharge/blob/master/src/main/java/frc/robot/subsystems/Vision.java

        // $IDO - Limelighthelpers
        // LimelightHelpers library: https://github.com/LimelightVision/limelightlib-wpijava
        // **** LimelightHelpers docs:
        // https://www.chiefdelphi.com/t/introducing-limelight-lib/425660?page=2
        // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib

        // Uses limelighthelpers:
        // https://github.com/6391-Ursuline-Bearbotics/2023-Swerverybot/blob/main/src/main/java/frc/robot/subsystems/Vision/Limelight.java

        // Limelight port forwarding for laptop use:
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/best-practices
        // Crosshair calibration:
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/crosshair

        tab.addDouble("ID", () -> getID());
        tab.addDouble("X Degrees", () -> getX())
                .withWidget(BuiltInWidgets.kDial)
                .withSize(2, 2)
                .withProperties(Map.of("min", -45, "max", 45));

        tab.addDouble("Y Degrees", () -> getY())
                .withWidget(BuiltInWidgets.kGyro)
                .withSize(2, 2)
                .withProperties(Map.of("Starting angle", 90.0));

        tab.addBoolean("Is Detecting", () -> isDetected());

        tab.addDouble("Distance Meters X", () -> getDistanceMetersX())
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 10));

        tab.addDouble("Distance Meters Y", () -> getDistanceMetersY())
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 10));

        tab.add("Field", m_fieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(3, 2);
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

    // $IDO - This seems like a strange way to see if any ID is detected
    public boolean isDetected() {
        return getX() + getY() + getArea() != 0;
    }

    public double getID() {
        return tableID.getDouble(0);
    }

    // $IDO - This isn't even used!
    public boolean isDetectedIDValid() {
        double myID = getID();
        if (Constants.VisionConstants.targetedIDList.contains(myID)) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Distance to April tag in meters Y.
     */
    // $IDO - This is where the Y height is being calculated
    public double getDistanceMetersY() {
        double angleToGoalRadians = limelightMountAngleRadiansY + getYRadians();
        double distanceFromLimelightToGoalMeters = (aprilTagHeightMeters
                - limelightLensHeightMeters) / (Math.tan(angleToGoalRadians) + EPSILON);
        return distanceFromLimelightToGoalMeters;
    }

    /**
     * Distance to April tag in meters X.
     */
    // $IDO - Is this right? It's calculating X meters from getDistanceMetersY()
    public double getDistanceMetersX() {
        double angleToGoalRadians = limelightMountAngleRadiansX + getXRadians();
        double distanceFromLimelightToGoalMeters = getDistanceMetersY()
                * Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
        LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-ramen");
        int numAprilTags = llresults.targetingResults.targets_Fiducials.length;
        System.out.println("numtags: " + numAprilTags);

        if (numAprilTags > 0) {

            Pose2d pose = llresults.targetingResults.targets_Fiducials[0]
                    .getRobotPose_FieldSpace2D();
            double x = pose.getTranslation().getX();
            double y = pose.getTranslation().getY();
            double rotation = pose.getRotation().getDegrees();
            System.out.println("x=" + x + ", y=" + y + ", rot=" + rotation);

            m_fieldSim.setRobotPose(pose);
        }
    }

    public void stopSystem() {
    }
}
