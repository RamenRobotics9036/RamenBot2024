package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers;

import java.util.Map;

public class VisionSystem extends SubsystemBase {
    private final double limelightMountAngleRadiansY = VisionConstants.limelightMountAngleRadiansY;
    private final double limelightMountAngleRadiansX = VisionConstants.limelightMountAngleRadiansX;

    private double m_speakerPositionY;
    private double m_speakerPositionX;
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

    private final double EPSILON = 0.0000001;
    private Pose2d m_fieldPose = new Pose2d();
    private double m_angleFromSpeaker = 0;

    public VisionSystem() {
        if (DriverStation.getAlliance().isPresent()) {
            m_speakerPositionY = (DriverStation.getAlliance().get().equals(Alliance.Red)) ? 14.4
                    : 0;
        }
        else {
            m_speakerPositionY = 0;
        }
        if (DriverStation.getAlliance().isPresent()) {
            m_speakerPositionX = (DriverStation.getAlliance().get().equals(Alliance.Red)) ? 5.3
                    : 5.3;
        }
        else {
            m_speakerPositionX = 0;
        }
        displayToShuffleBoard();
        LimelightHelpers
                .setCameraPose_RobotSpace(
                        VisionConstants.limelightName,
                        0,
                        0,
                        0.66,
                        0,
                        15,
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

        tab.addDouble("ABC DAVID Y Meters to Target", () -> getSpeakerYDistance());
        tab.addDouble("ABC DAVID X Meters to Target", () -> getSpeakerXDistance());
        tab.addDouble("ABC DAVID Degrees to Target", () -> getSpeakerRotation().getDegrees());
        tab.addDouble("ABC DAVID Robot Position Y", () -> m_fieldPose.getX());
        tab.addDouble("ABC DAVID Speaker Position Y", () -> m_speakerPositionY);

        tab.addBoolean("Is Detecting", () -> isDetected())
                .withPosition(0, 0);

        tab.addInteger("Num tags", () -> m_numTags[0])
                .withPosition(1, 0);

        tab.addDouble("ID", () -> getID())
                .withPosition(2, 0);

        tab.addDouble("X Degrees", () -> getX())
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(1, 2)
                .withSize(2, 2);

        tab.addDouble("Y Degrees", () -> getY())
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(3, 2)
                .withSize(2, 2)
                .withProperties(Map.of("Starting angle", 270.0));

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

    @Override
    public void periodic() {
        LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-ramen");

        int numAprilTags = llresults.targetingResults.targets_Fiducials.length;
        m_numTags[0] = numAprilTags;

        // Only accurate if 2 tags are detected.
        if (numAprilTags >= 2) {
            m_fieldPose = llresults.targetingResults.getBotPose2d_wpiBlue();
            m_fieldSim.setRobotPose(m_fieldPose);
            m_isTargetDetected = true;

            for (LimelightTarget_Fiducial tag : llresults.targetingResults.targets_Fiducials) {
                if (tag.fiducialID == 4 || tag.fiducialID == 7) {
                    m_angleFromSpeaker = tag.tx;
                }
            }
        }
        else {
            // Reset robot picture on the field
            m_fieldPose = new Pose2d();
            m_isTargetDetected = false;
            m_angleFromSpeaker = 0;
        }
    }

    public double getSpeakerYDistance() {
        if (m_isTargetDetected) {
            return Math.abs(m_fieldPose.getX() - m_speakerPositionY);
        }
        else {
            return 0;
        }
    }

    public double getSpeakerXDistance() {
        if (m_isTargetDetected) {
            return Math.abs(m_fieldPose.getY() - m_speakerPositionX);
        }
        else {
            return 0;
        }
    }

    public Rotation2d getSpeakerRotation() {
        if (m_isTargetDetected) {
            return Rotation2d.fromDegrees(m_angleFromSpeaker);
        }
        else {
            return new Rotation2d();
        }
    }

    public boolean getIsDetecting() {
        return m_isTargetDetected;
    }

    public void stopSystem() {
    }
}
