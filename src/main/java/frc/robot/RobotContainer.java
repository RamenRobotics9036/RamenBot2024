package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.HookSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.commands.AmpLightCommand;
import frc.robot.commands.IntakeRevCommand;
import frc.robot.commands.LEDResetCommand;
import frc.robot.commands.PullBackCommand;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.commands.StayCommand;
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
    private HookSystem m_hookSystem = new HookSystem(m_armController);
    private LEDSystem m_LEDSystem = new LEDSystem(m_intakeSystem);

    SendableChooser<String> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        m_autoChooser.setDefaultOption("3 Note Auto High", "MID-TOP 3 NOTE");
        m_autoChooser.addOption("3 Note Auto Low", "MID-BOTTOM 3 NOTE");
        m_autoChooser.addOption("Low Score Escape", "BOTTOM LEAVE 1 NOTE");
        m_autoChooser.addOption("High Score Escape", "TOP LEAVE 1 NOTE");

        Shuffleboard.getTab("Auto").add(m_autoChooser);

        double waitTime = 0.2;
        // initShuffleBoard();

        // I will probably need to add a timer or maybe I can do that in Path Planner
        NamedCommands.registerCommand(
                "Set Arm To Ground",
                new SetArmToAngleCommand(m_armSystem, SetArmConstants.armMin));
        NamedCommands.registerCommand(
                "Set Arm To Shoot",
                // new SetArmToAngleCommand(m_armSystem,
                // PresetConstants.speakerPresetAngleRadians));
                new ParallelDeadlineGroup(new SetArmToAngleCommand(m_armSystem,
                        PresetConstants.speakerPresetAngleAutoRadians),
                        new StayCommand(m_swerveDrive)));
        NamedCommands.registerCommand(
                "Shoot Note",
                new ParallelDeadlineGroup(
                        new PullBackCommand(m_intakeSystem)
                                .andThen(new WaitCommand(waitTime))
                                .andThen(
                                        new IntakeRevCommand(m_intakeSystem, m_shooterSystem,
                                                m_armController)),
                        new StayCommand(m_swerveDrive)));
    }

    public void scheduleAutonomousCommand() {

        // Choose which Field relative to use
        // use a sendable chooser for which gyro to reset to

        // NOTE Field relative is dependent both on which alliance you are on, but also what part of
        // the subwoofer you are on.

        // m_swerveDrive.resetGyroFieldRelativeBlueMid();
        m_swerveDrive.resetPose(
                new Pose2d(new Translation2d(),
                        Rotation2d.fromRadians(m_swerveDrive.getAnglePosition())));

        AutoBuilder.configureHolonomic(
                m_swerveDrive::getPoseMeters,
                m_swerveDrive::resetPose,
                m_swerveDrive::getSpeeds,
                m_swerveDrive::driveFromChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(19, 0, 0),
                        new PIDConstants(0, 0, 0),
                        SwerveSystemConstants.maxSpeedMetersPerSecondAuto,
                        m_swerveDrive.getDriveBaseRadius(),
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red),
                m_swerveDrive);

        String autoName = m_autoChooser.getSelected();
        Command auto = new PathPlannerAuto(autoName);
        auto.schedule();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Arm").addDouble(
                "Angle to Shoot",
                () -> m_armSystem.getShootingAngle(m_visionSystem.getDistanceMetersY())
                        + ShooterConstants.shootOffsetLimeLight);
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {

        // ARM CONTROLLER BINDINGS

        // Push note piece back on start up. May not need to happen when reflectometer is used.
        double waitTime = 0.1; // Was 0.2, revert back if it does not work (I THINK I COULD PUT IT
                               // TO .01 BECAUSE THE MOTORS STOP RUNNING WHICH MEANS IT TAKES TIME
                               // TO REV UP ANYWAYS)
        new Trigger(() -> m_armController.getAButton()).onTrue(
                new PullBackCommand(m_intakeSystem)
                        .andThen(new WaitCommand(waitTime))
                        .andThen(
                                new IntakeRevCommand(m_intakeSystem, m_shooterSystem,
                                        m_armController)));

        // Amp Preset
        new Trigger(() -> m_armController.getXButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.ampPresetAngleRadians));

        // Sub-woofer Preset
        new Trigger(() -> m_armController.getYButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.speakerPresetAngleRadians));

        // One robot away preset
        new Trigger(() -> m_armController.getBButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem,
                        PresetConstants.speakerPresetAngleAutoOneRobotAwayRadians));

        // DRIVE CONTROLLER BINDINGS

        new Trigger(() -> m_driveController.getBButton()).onTrue(
                // AMP LIGHT COMMAND
                new AmpLightCommand(m_LEDSystem));

        new Trigger(() -> m_driveController.getYButton()).onTrue(
                // RESETS LED JUST IN CASE THE CODE IS NOT RIGHT
                new LEDResetCommand(m_LEDSystem));

        // Auto-align
        // new Trigger(() -> m_driveController.getAButton()).onTrue(
        // new VisionAutoAlignCommand(m_swerveDrive, m_visionSystem));

    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_visionSystem.stopSystem();

        m_shooterSystem.stopSystem();
        m_armSystem.stopSystem();
        m_intakeSystem.stopSystem();
        m_hookSystem.stopSystem();
    }
}
