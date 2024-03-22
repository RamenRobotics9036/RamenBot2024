package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    private double m_speakerPosition;
    private boolean m_isTargetDetected = false;

    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    private NetworkTable limelightTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.limelightName);
    private NetworkTableEntry tableX = limelightTable.getEntry("tx");
    private NetworkTableEntry tableY = limelightTable.getEntry("ty");
    private NetworkTableEntry tableArea = limelightTable.getEntry("ta");
    private NetworkTableEntry tableID = limelightTable.getEntry("tid");

    private final Field2d m_fieldSim = new Field2d();
    private int[] m_numTags = {
            0
    };

    private int priorityTag;

    private final double EPSILON = 0.0000001;
    private Pose2d m_fieldPose = new Pose2d();

    public VisionSystem() {
        if (DriverStation.getAlliance().isPresent()) {
            m_speakerPosition = (DriverStation.getAlliance().get().equals(Alliance.Red)) ? 16.47
                    : 2.227;
            priorityTag = (DriverStation.getAlliance().get().equals(Alliance.Red)) ? 4
                    : 7;
        }
        else {
            m_speakerPosition = 0;
            priorityTag = 7;
        }
        displayToShuffleBoard();
        LimelightHelpers.setPriorityTagID(VisionConstants.limelightName, priorityTag);
        LimelightHelpers
                .setCameraPose_RobotSpace(
                        VisionConstants.limelightName,
                        0,
                        0.2,
                        0.42,
                        0,
                        26,
                        0);
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

        // tab.addDouble("ABC DAVID Meters to Target", () -> getSpeakerYDistance()).withPosition(
        // 0,
        // 0);
        tab.addDouble("ABC DAVID Robot Position Y", () -> m_fieldPose.getX()).withPosition(0, 1);
        tab.addDouble("ABC DAVID Speaker Position Y", () -> m_speakerPosition).withPosition(0, 2);

        // tab.addDouble("ABC DAVID Meters to Subwoofer", () -> getSpeakerYDistance() - 1.07);

        tab.addBoolean("Is Detecting", () -> isDetected())
                .withPosition(1, 0);

        tab.addInteger("Num tags", () -> m_numTags[0])
                .withPosition(1, 1);

        // tab.addInteger("Priority Tag", () -> priorityTag)
        // .withPosition(1, 2);

        // tab.addDouble("ID", () -> getID())
        // .withPosition(2, 0);

        // tab.addDouble("X Degrees", () -> getX())
        // .withWidget(BuiltInWidgets.kGyro)
        // .withPosition(1, 2)
        // .withSize(2, 2);

        // tab.addDouble("Y Degrees", () -> getY())
        // .withWidget(BuiltInWidgets.kGyro)
        // .withPosition(3, 2)
        // .withSize(2, 2)
        // .withProperties(Map.of("Starting angle", 270.0));

        // tab.addDouble("Meters to target", () -> getDistanceMetersY())
        // .withWidget(BuiltInWidgets.kNumberBar)
        // .withPosition(3, 0)
        // .withSize(2, 1)
        // .withProperties(Map.of("min", 0, "max", 10));

        tab.add("Field", m_fieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withPosition(5, 2)
                .withSize(5, 3);
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

    public void updatePose() {
        LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-ramen");

        int numAprilTags = llresults.targetingResults.targets_Fiducials.length;
        m_numTags[0] = numAprilTags;

        // Only accurate if 2 tags are detected.
        if (numAprilTags >= 1) {
            m_fieldPose = llresults.targetingResults.getBotPose2d_wpiBlue();
            m_fieldSim.setRobotPose(m_fieldPose);
            m_isTargetDetected = true;
        }
        else {
            // Reset robot picture on the field
            m_fieldPose = new Pose2d();
            m_isTargetDetected = false;
        }
    }

    public double getSpeakerYDistance() {
        updatePose();
        if (m_isTargetDetected) {
            return m_speakerPosition - m_fieldPose.getX();
        }
        else {
            return 0;
        }
    }

    public void stopSystem() {
    }
}
