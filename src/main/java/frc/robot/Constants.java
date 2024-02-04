package frc.robot;

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

        public static final double controllerDeadbandPercent = 0.03; // 0.2
    }

    /**
     * Constants for the swerve system.
     */
    public static class SwerveSystemConstants {
        public static final boolean isPIDTuning = true;

        public static final double frameDistanceToModulesMeters = 0.28;
        public static final double wheelRadiusMeters = 0.051;

        public static final int gyroCanID = 7;

        // PLEASE do not *check-in* a higher value than 0.5, since robot features are still being
        // tested.
        // If you need to raise it higher, change it on your local code only
        public static final double maxOutputPercentage = 0.25;

        // PID tunes for 51.5 pounds
        public static final double drivingPID_P = 8;
        public static final double drivingPID_I = 0;
        public static final double drivingPID_D = 3;

        // PID tunes for 51.5 pounds
        public static final double turningPID_P = 1.45;
        public static final double turningPID_I = 0;
        public static final double turningPID_D = 0.32;

        public static final double drivingFeedForward_S = 0.11095;
        public static final double drivingFeedForward_V = 2.3901;
        public static final double drivingFeedForward_A = 0.1212;

        public static final double maxSpeedMetersPerSecond = 3;

        // // 2.5 (THESE VALUES ARE PRETTY RANDOM) was * 10
        public static final double maxAngularSpeed = 0.574 * 10;

        // 2 * Math.PI; (THESE VALUES ARE PRETTY RANDOM)
        public static final double maxAngularAcceleration = 0.574 * 100;

        public static final int swerveMotorCurrentLimit = 20;

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

            // NOTE: This makes the front of the robot the right side. (the side of the radio), but
            // it does not neceassrily matter because of field relativity
            public static final double frontLeftOffset = -Math.PI + .1 + rotationOffset;
            public static final double backLeftOffset = 0.744 + Math.PI + rotationOffset;
            public static final double frontRightOffset = (Math.PI / 4) + ((Math.PI * 2) - 4.708)
                    + 0.2 + rotationOffset;
            public static final double backRightOffset = 0.928 + rotationOffset;

        }
    }

    public static class VisionConstants {
        /**
         * Angle of camera pointing upwards.
         */
        public static final double limelightMountAngleRadiansY = 0;
        /**
         * Angle of camera pointing side-to-side.
         */
        public static final double limelightMountAngleRadiansX = 0;

        public static final double limelightLensHeightMeters = 0.38;
        public static final double aprilTagHeightMeters = 0.9;

        public static final String limelightName = "limelight-ramen";
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
            public static final double percentPower = 1;

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
            public static final double PID_P = 1;
            public static final double PID_I = 0;
            public static final double PID_D = 1;

            public static final double percentPower = 0.2;

            public static final double errorMargin = 0.1;

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

            public static final double targetDistanceMeters = 2;

            public static final double translationXPID_P = 0.5;
            public static final double translationXPID_I = 0;
            public static final double translationXPID_D = 0;

            public static final double translationYPID_P = 0.45;
            public static final double translationYPID_I = 0;
            public static final double translationYPID_D = 0;

            public static final double rotationPID_P = 1 / 1000;
            public static final double rotationPID_I = 0;
            public static final double rotationPID_D = 0;
        }
    }

    public static class ShooterConstants {
        public static final int shooterLeftMotorID = 18;
        public static final int shooterRightMotorID = 19;
        public static final double maxOutputPercent = 1;
        public static final double shooterSpeed = 0.7;
    }

    public static class ArmConstants {
        public static final int armMotorID = 22;
        public static final int armEncoderChannel = 0;
        public static final double armSpeed = 0.1;
        public static final double armLegnth = 25.4;
        public static final double centerSpeakerHeight = 80.25;
        public static final double pivotHeightOverGround = 0.279;// The pivot height over ground in
                                                                 // meters.
        public static final double shootToPivotRadius = 0.549;// Radius from shooting point to pivot
                                                              // point in meters.
        public static final double armAngleOffsetHorizontal = 0;// Offset bewteen sensor to
                                                                // horizontal axis of arm in degrees

        public static final double armSpeedFast = 1;
        public static final double maxOutputPercent = 0.2;

    }

    public static class IntakeConstants {
        public static final int intakeMotorLeftID = 20;
        public static final int intakeMotorRightID = 21;
        public static final int reflectChannel = 4;
        public static final double intakeSpeed = 0.4;
        public static final double maxOutputPercent = 0.4;
    }

    public static class RevConstants {
        public static final double revTime = 0.2;
    }

    public static class TestConstants {
        public static final double testTime = 0.5;
        public static final double testSpeed = 0.2;
        public static final double errorMargin = 0.1;
    }
}
