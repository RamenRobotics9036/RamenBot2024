package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveDriveSystemAbstract extends SubsystemBase {
    public abstract void toTeleop();

    public abstract void toAuto();

    public abstract void setFieldRelative(boolean fieldRelative);

    public abstract void drive(double xspeed, double yspeed, double rot);

    public abstract void drive(double xspeed, double yspeed, double rot, boolean fieldRelative);

    public abstract double getxPosition();

    public abstract double getyPosition();

    public abstract boolean resetGyroFieldRelative();

    public abstract boolean resetGyroFieldRelativeRedTop();

    public abstract boolean resetGyroFieldRelativeRedBottom();

    public abstract boolean resetGyroFieldRelativeBlueTop();

    public abstract boolean resetGyroFieldRelativeAuto();

    public abstract boolean resetGyroFieldRelativeAutoRed();

    public abstract boolean resetGyroFieldRelativeBlueBottom();

    public abstract double getAnglePosition();

    public abstract Rotation2d getRotation2d();

    public abstract double getFrontLeftTurnEncoder();

    public abstract double getBackLeftTurnEncoder();

    public abstract double getFrontRightTurnEncoder();

    public abstract double getBackRightTurnEncoder();

    public abstract double getFrontLeftDriveEncoder();

    public abstract double getBackLeftDriveEncoder();

    public abstract double getFrontRightDriveEncoder();

    public abstract double getBackRightDriveEncoder();

    public abstract double getFrontLeftDriveVelocity();

    public abstract double getBackLeftDriveVelocity();

    public abstract double getFrontRightDriveVelocity();

    public abstract double getBackRightDriveVelocity();

    public abstract void setStatus(boolean[] status);

    public abstract ChassisSpeeds getSpeeds();

    public abstract double getDriveBaseRadius();

    public abstract void resetPose(Pose2d pose);

    public abstract void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds);

    public abstract Pose2d getPoseMeters();

    public abstract void stopSystem();
}
