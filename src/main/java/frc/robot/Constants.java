package frc.robot;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical.
 */
public class Constants {
    /**
     * Constants for the joystick.
     */
    public static class OperatorConstants {
        public static final int driveControllerPort = 0;
        public static final int armControllerPort = 1;
        public static final double controllerExpo = 4;

        // Change this variable if you want to change exponent
        public static final double controllerExpoRatio = 0.6;
        public static final double controllerExpoRatioRight = 0.85;

        public static final double controllerDeadbandPercent = 0.06; // 0.2

        public static final int kLEDLightsChannel = 0;
        public static final int kLEDLightsLength = 34;
    }

    /**
     * Constants for the swerve system.
     */
    public static class SwerveSystemConstants {
        public static final boolean isPIDTuning = true;

        public static final double frameDistanceToModulesMeters = 0.5461;
        public static final double wheelRadiusMeters = 0.0508; // 2 inches

        public static final int gyroCanID = 7;

        // PLEASE do not *check-in* a higher value than 0.5, since robot features are still being
        // tested.
        // If you need to raise it higher, change it on your local code only
        public static final double maxOutputPercentage = 1;
        public static final double autoSpeed = 0.6; // Trying to lower max auto speed

        // PID tunes for 51.5 pounds
        public static final double drivingPID_P = 5;
        public static final double drivingPID_I = 0;
        public static final double drivingPID_D = 0.6;

        // PID tunes for 51.5 pounds
        public static final double turningPID_P = 1.45;
        public static final double turningPID_I = 0;
        public static final double turningPID_D = 0.32;

        public static final double drivingFeedForward_S = 0.11095;
        public static final double drivingFeedForward_V = 2.3901;
        public static final double drivingFeedForward_A = 0.1212;

        public static final double maxSpeedMetersPerSecond = 5.06;
        public static final double maxSpeedMetersPerSecondAuto = 5.06;

        // // 2.5 (THESE VALUES ARE PRETTY RANDOM) was * 10
        public static final double maxAngularSpeed = 0.574 * 10;

        // 2 * Math.PI; (THESE VALUES ARE PRETTY RANDOM)
        public static final double maxAngularAcceleration = 0.574 * 100;

        public static final int swerveMotorCurrentLimit = 40;

        public static final double driveMotorGearBoxRatio = 1 / 6.12;
        public static final double turnMotorGearBoxRatio = 12.8;

        /**
         * Constants for the swerve devices.
         */
        public static class SwerveSystemDeviceConstants {
            public static final int frontLeftDriveMotorID = 11;
            public static final int frontLeftTurnMotorID = 10;

            public static final int frontRightDriveMotorID = 17;
            public static final int frontRightTurnMotorID = 16;

            public static final int backLeftDriveMotorID = 13;
            public static final int backLeftTurnMotorID = 12;

            public static final int backRightDriveMotorID = 15;
            public static final int backRightTurnMotorID = 14;

            public static final int frontLeftTurnEncoderChannel = 0;
            public static final int frontRightTurnEncoderChannel = 3;

            public static final int backLeftTurnEncoderChannel = 1;
            public static final int backRightTurnEncoderChannel = 2;

            // Math.PI/2 offsets all wheels by 90 degrees which lets the rotation work. Before, the
            // wheels were facing inwards which means when the robot tried to rotate,
            // all the wheels were facing inwards which would cancel out the rotation.
            // The wheels go from basically creating an X to creating an O because now all wheels
            // are facing outwards when rotating
            public static final double rotationOffset = Math.PI / 2;

            // Different Swerve

            public static final double frontLeftOffsetSwerveA = -Math.PI + .1 + rotationOffset;
            public static final double backLeftOffsetSwerveA = 0.744 + Math.PI + rotationOffset;

            public static final double frontRightOffsetSwerveA = (Math.PI / 4)
                    + ((Math.PI * 2) - 4.708)
                    + 0.2 + rotationOffset;
            public static final double backRightOffsetSwerveA = 0.928 + rotationOffset;

            // NOTE: This makes the front of the robot the right side. (the side of the radio), but
            // it does not neceassrily matter because of field relativity
            public static final double frontLeftOffsetSwerveB = (Math.PI * 1.5) - 3.314
                    + rotationOffset;
            public static final double backLeftOffsetSwerveB = frontRightOffsetSwerveA + Math.PI;
            public static final double frontRightOffsetSwerveB = (Math.PI * 1.5) - 1.858 + Math.PI
                    + rotationOffset;
            public static final double backRightOffsetSwerveB = backRightOffsetSwerveA;

        }
    }

    public static class VisionConstants {
        /**
         * Angle of camera pointing upwards.
         */
        public static final double limelightMountAngleRadiansY = Math.toRadians(18.5);

        /**
         * Angle of camera pointing side-to-side.
         */
        public static final double limelightMountAngleRadiansX = 0;

        public static final double limelightLensHeightMeters = 0.51;
        public static final double aprilTagHeightMeters = 1.47;

        public static final String limelightName = "limelight-ramen";

        // $IDO - These are the valid targetIDs we target
        public static final ArrayList<Double> targetedIDList = new ArrayList<Double>(
                Arrays.asList(
                        1.0,
                        2.0,
                        3.0,
                        4.0,
                        5.0,
                        6.0,
                        7.0,
                        8.0,
                        9.0,
                        10.0,
                        11.0,
                        12.0,
                        13.0,
                        14.0,
                        15.0,
                        16.0));

        /*
         * Distance -> Angle Pairs
         * Distance is in meters
         * Measure every lookUpTableDistance
         */
        @SuppressWarnings("LineLengthCheck")
        public static final ArrayList<Pair<Double, Double>> angleLookUpTable = new ArrayList<Pair<Double, Double>>(
                Arrays.asList(
                        new Pair<Double, Double>(0., PresetConstants.speakerPresetAngleRadians),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 1, 4.97),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 2, 4.89),

                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 3, 4.81),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 4, 4.77),

                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 5, 4.74),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 6, 4.72),

                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 7, 4.71),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 8, 4.705),

                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 9, 4.7),
                        new Pair<Double, Double>(ArmConstants.lookUpTableDistance * 10, 4.695)));
    }

    /**
     * Constants for the commands.
     */
    public static class CommandsConstants {
        /**
         * Constants for command to drive to a specific location.
         */
        public static class SetAxisConstants {
            public static final double errorMarginXY = 0.05;
            public static final double errorMarginRot = 0.02;
            public static final double percentPower = 0.5;

            public static final double translationPID_P = 1;
            public static final double translationPid_I = 0;
            public static final double translationPID_D = 0;

            public static final double rotationPID_P = 0.03;
            public static final double rotationPID_I = 0;
            public static final double rotationPID_D = 0;
            public static final double timeLimit = 8.0;
            public static final double armExtendTimeLimit = 5.0;
        }

        public static class SetArmConstants {

            public static final double armMax = 3.7;
            public static final double armMin = 5.45;
            public static final double maxTime = 3;
            public static final double PID_P = 1;
            public static final double PID_I = 0;
            public static final double PID_D = 1;

            public static final double percentPower = 0.2;

            public static final double errorMargin = 0.01;

        }

        public static class IntakeReleaseConstants {
            public static final double maxTime = 0.35;
        }

        public static class VisionAutoAlignConstants {
            public static final double errorMarginDistanceX = 0.08;
            public static final double errorMarginDistanceY = 0.04;
            public static final double errorMarginRot = 2;
            public static final double percentPower = 1;

            public static final double timeLimit = 12.0;

            public static final double targetDistanceMeters = 3.5;

            public static final double translationXPID_P = 0.05;
            public static final double translationXPID_I = 0;
            public static final double translationXPID_D = 0;

            public static final double translationYPID_P = 0.3;
            public static final double translationYPID_I = 0;
            public static final double translationYPID_D = 0;

            public static final double rotationPID_P = 1 / 10_000;
            public static final double rotationPID_I = 0;
            public static final double rotationPID_D = 0;

            public static final double errorMarginVelocityX = 0.1;
            public static final double errorMarginVelocityY = 0.1;
            public static final double errorMarginVelocityRot = 0.05;
        }
    }

    public static class ShooterConstants {
        public static final double shootOffsetLimeLight = 0;
        public static final int shooterLeftMotorID = 18;
        public static final int shooterRightMotorID = 19;
        public static final double maxOutputPercent = 1;
        public static final double shooterSpeed = 0.7;
        public static final double lowShooterSpeed = 0.2;
        public static final double pullBackAmount = 4;

        public static boolean shouldCharge = false;
    }

    public static class ArmConstants {
        public static final double lookUpTableDistance = 0.23;
        public static final double distanceToPivot = 0.5;
        public static final int smartCurrentLimit = 40;
        public static final double gearRatio = 60 / 12;
        public static final int armMotorIDFollower = 22;
        public static final int armMotorIDLeader = 23;
        public static final int armEncoderChannel = 2;
        public static final double armSpeed = 0.1;
        public static final double armLegnth = 25.4;
        public static final double centerSpeakerHeight = 2.038;

        // The pivot height over ground in meters.
        public static final double pivotHeightOverGround = 0.29;

        // Radius from shooting point to pivot point in meters.
        public static final double shootToPivotRadius = 0.549;

        // Offset bewteen sensor to horizontal axis of arm in degrees
        public static final double armAngleOffsetHorizontal = 0;

        public static final double armSpeedFast = 1;
        public static final double maxOutputPercent = 0.8;
        public static final double maxOutputPercentTeleop = 0.8;
    }

    public static class IntakeConstants {
        public static final double pullBackAmount = 4.5;
        public static final double pullBackSpeed = -0.01;
        public static final double intakeSpeed = 0.3;

        public static final int smartCurrentLimit = 20;
        public static final int intakeMotorLeftID = 20;
        public static final int intakeMotorRightID = 21;
        public static final int reflectChannel = 4;
        public static final double maxOutputPercent = 0.7;

        public static double speed = 0;
    }

    public static class RevConstants {
        public static final double revTime = 0.5;
        public static final double maxTime = 0.8;
    }

    public static class TestConstants {
        public static final double testTime = 0.5;
        public static final double testSpeed = 0.2;
        public static final double errorMargin = 0.1;
    }

    public static class PresetConstants {
        public static final double ampPresetAngleRadians = 3.7;

        // (TESTED, WORKS VERY WELL) a little lower because our current auto is consistently
        // undershooting distance which means it needs a lower angle
        public static final double speakerPresetAngleAutoRadians = 5.03;
        public static final double speakerPresetAngleRadians = 5.03;
        public static final double shooterSpeed = 0.7;
        public static final double speakerPresetAngleAutoOneRobotAwayRadians = 4.907;
    }

    public static class HookConstants {
        public static final int leftHookCANId = 24;
        public static final int rightHookCANId = 25;
        public static final double maxHeight = 4.6;
        public static final double minHeight = 0;
        public static final double maxOutputPercent = .8;
    }

    public static class GrabChainConstants {
        public static final double hookSpeed = 0.2;
        public static final double swerveSpeed = 0.3;
        public static final double hookRotationsNeeded = 5;
        public static final double hookRotationsNeededFinal = 5;
        public static final double swerveRotationsNeeded = 5;
        public static final double maxTime = 10;
    }

    public static class RotateSwerveConstants {
        public static final double kP = 0.015;
        public static final double translationDeadband = 0.01;
        public static final double velocityDeadband = 0.01;
    }

}
