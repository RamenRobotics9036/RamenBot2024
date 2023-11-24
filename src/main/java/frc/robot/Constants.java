package frc.robot;

public class Constants {
    public static class OperatorConstants {
        public static final int driveControllerPort = 0;
        public static final double controllerDeadband = 0.05;
    }

    public static class SwerveSystemConstants {
        public static final double frameDimension = 0.5;
        public static final double wheelRadius = 0.05;
        public static final int gyroChannel = 0;

        public static final double drivingPID_P = 1;
        public static final double drivingPID_I = 0;
        public static final double drivingPID_D = 0;

        public static final double turningPID_P = 1;
        public static final double turningPID_I = 0;
        public static final double turningPID_D = 0;

        public static final double turningFeedForward_S = 1;
        public static final double turningFeedForward_V = 3;

        public static final double drivingFeedForward_S = 1;
        public static final double drivingFeedForward_V = 0.5;

        public static final double maxSpeed = 3.0;
        public static final double maxAngularSpeed = Math.PI;
        public static final double maxAngularAcceleration = 2 * Math.PI;

        public static final int swerveMotorCurrentLimit = 20;

        public static class SwerveSystemDeviceConstants {
            public static final int frontLeftDriveMotorID = 10;
            public static final int frontLeftTurnMotorID = 11;

            public static final int frontRightDriveMotorID = 12;
            public static final int frontRightTurnMotorID = 13;

            public static final int backLeftDriveMotorID = 14;
            public static final int backLeftTurnMotorID = 15;

            public static final int backRightDriveMotorID = 16;
            public static final int backRightTurnMotorID = 17;

            public static final int frontLeftDriveEncoderChannel = 0;
            public static final int frontLeftTurnEncoderChannel = 1;

            public static final int frontRightDriveEncoderChannel = 2;
            public static final int frontRightTurnEncoderChannel = 3;

            public static final int backLeftDriveEncoderChannel = 4;
            public static final int backLeftTurnEncoderChannel = 5;

            public static final int backRightDriveEncoderChannel = 6;
            public static final int backRightTurnEncoderChannel = 7;
        }
    }

    public static class WatchTimesConstants {
        public static final double turnSwerveWatchTime = 3;
    }

    public static class CommandConstants {
        public static final double swerveRotateDegrees = 90;
        public static final double swerveRotateSpeed = 0.2;
    }
}
