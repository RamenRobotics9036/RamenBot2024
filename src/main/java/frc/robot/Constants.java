package frc.robot;

/**
 * Lots of constants.
 */
public class Constants {
  /**
   * Constants for the operator interface.
   */
  public static class OperatorConstants {
    public static final int driveControllerPort = 0;
    public static final double controllerDeadbandPercent = 0.08;
  }

  /**
   * Constants for the swerve system.
   */
  public static class SwerveSystemConstants {
    public static final double frameDistanceToModulesMeters = 0.28;
    public static final double wheelRadiusMeters = 0.051;

    public static final int gyroCanID = 7;
    public static final double maxOutputPercentage = 1;

    public static final double drivingPID_P = 0.05;
    public static final double drivingPID_I = 0;
    public static final double drivingPID_D = 0;

    public static final double turningPID_P = 2.82;
    public static final double turningPID_I = 0;
    public static final double turningPID_D = 1;

    public static final double drivingFeedForward_S = 0.11095;
    public static final double drivingFeedForward_V = 2.3901;
    public static final double drivingFeedForward_A = 0.1212;

    public static final double maxSpeedMetersPerSecond = 2.0;
    public static final double maxAngularSpeed = Math.PI;
    public static final double maxAngularAcceleration = 2 * Math.PI;

    public static final int swerveMotorCurrentLimit = 20;

    public static final double driveMotorGearBoxRatio = 1 / 6.12;
    public static final int turnMotorGearBoxRatio = 1;

    /**
     * Constants for the swerve system's devices.
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

      public static final double frontLeftOffset = 1.509;
      public static final double backLeftOffset = 3.981 - Math.PI;
      public static final double frontRightOffset = 2.129;
      public static final double backRightOffset = 0.636 + Math.PI;
    }
  }
}
