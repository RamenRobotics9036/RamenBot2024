package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.IntakeRevCommand;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.commands.SetIntakeSpeedCommand;
import frc.robot.commands.SetShooterSpeedCommand;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.AppliedController;

/**
 * RobotContainer.
 */
public class RobotContainer {
    private final AppliedController m_driveController = new AppliedController(
            OperatorConstants.driveControllerPort);
    private final AppliedController m_armController = new AppliedController(
            OperatorConstants.armControllerPort);

    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);
    private VisionSystem m_visionSystem = new VisionSystem();

    private ShooterSystem m_shooterSystem = new ShooterSystem();
    private ArmSystem m_armSystem = new ArmSystem(m_armController);
    private IntakeSystem m_intakeSystem = new IntakeSystem();

    public RobotContainer() {
        initShuffleBoard();
    }

    public void scheduleAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        new FollowPathHolonomic(
                path,
                m_swerveDrive::getPoseMeters,
                m_swerveDrive::getSpeeds,
                m_swerveDrive::driveFromChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        SwerveSystemConstants.maxSpeedMetersPerSecond,
                        m_swerveDrive.getDriveBaseRadius(),
                        new ReplanningConfig()),
                () -> false,
                m_swerveDrive).schedule();
    }

    private void initShuffleBoard() {
        Shuffleboard.getTab("Arm")
                .addDouble(
                        "Angle to Shoot",
                        () -> m_armSystem.getShootingAngle(
                                m_visionSystem.getDistanceMetersY())
                                + ShooterConstants.shootOffsetLimeLight);
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {
        // Push note piece back on start up. May not need to happen when reflectometer is used.
        double pullBackNoteTime = 0.2;
        double pullBackNoteSpeed = 0.2;
        double waitTime = 0.2;
        new Trigger(() -> m_armController.getAButton()).onTrue(
                new ParallelCommandGroup(
                        new SetShooterSpeedCommand(m_shooterSystem, pullBackNoteTime,
                                -pullBackNoteSpeed),
                        new SetIntakeSpeedCommand(m_intakeSystem, pullBackNoteTime,
                                pullBackNoteSpeed))
                        .andThen(new WaitCommand(waitTime))
                        .andThen(
                                new IntakeRevCommand(m_intakeSystem, m_shooterSystem,
                                        m_armController)));

        new Trigger(() -> m_armController.getBButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, m_armSystem.getShootingAngle(
                        m_visionSystem.getDistanceMetersY())
                        + ShooterConstants.shootOffsetLimeLight).andThen(
                                new WaitCommand(waitTime).andThen(
                                        new ParallelCommandGroup(
                                                new SetShooterSpeedCommand(m_shooterSystem,
                                                        pullBackNoteTime,
                                                        -pullBackNoteSpeed),
                                                new SetIntakeSpeedCommand(m_intakeSystem,
                                                        pullBackNoteTime,
                                                        pullBackNoteSpeed))
                                                .andThen(
                                                        new IntakeRevCommand(m_intakeSystem,
                                                                m_shooterSystem,
                                                                m_armController)))));

        // Amp Preset
        new Trigger(() -> m_armController.getXButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.ampPresetAngleRadians));

        // Sub-woofer Preset
        new Trigger(() -> m_armController.getYButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.speakerPresetAngleRadians));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_visionSystem.stopSystem();

        m_shooterSystem.stopSystem();
        m_armSystem.stopSystem();
        m_intakeSystem.stopSystem();
    }
}
