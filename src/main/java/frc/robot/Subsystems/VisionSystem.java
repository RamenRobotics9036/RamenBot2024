package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {
    Pose2d pose;
    private final double limelightMountAngleRadians = VisionConstants.limelightMountAngleRadians;
    private final double limelightLensHeightMeters = VisionConstants.limelightLensHeightMeters;
    private final double aprilTagHeightMeters = VisionConstants.aprilTagHeightMeters;

    public VisionSystem() {
        displayToShuffleBoard();
    }

    private void displayToShuffleBoard() {
        ShuffleboardLayout visionLayout = Shuffleboard.getTab("Vision")
                .getLayout("April Tags", BuiltInLayouts.kList);
        visionLayout.addDouble("Distance Meters", () -> getDistanceMetersToGoal());
        visionLayout.addDouble("X Displacement", () -> getX());
        visionLayout.addDouble("Y Displacement", () -> getY());
        visionLayout.addDouble("Angle Displacement", () -> getAngleRadians());
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public double getAngleRadians() {
        return pose.getRotation().getRadians();
    }

    public double getAngleDegrees() {
        return pose.getRotation().getDegrees();
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getDistanceMetersToGoal() {
        double angleToGoalRadians = limelightMountAngleRadians + getAngleRadians();
        double distanceFromLimelightToGoalMeters = (aprilTagHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }

    @Override
    public void periodic() {
        pose = LimelightHelpers.getBotPose2d("");
    }

    public void stopSystem() { 
    }
}
