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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.commands.AmpLightCommand;
import frc.robot.commands.IntakeRevCommand;
import frc.robot.commands.LEDResetCommand;
import frc.robot.commands.PullBackCommand;
import frc.robot.commands.RevCommandAmp;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.commands.SetAxisCommand;
import frc.robot.commands.StayCommand;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.HookSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.sim.DefaultSimLayout;
import frc.robot.sim.PopulateSimShuffleboard;
import frc.robot.sim.SwerveDriveSystemSim;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.HookSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.AppliedController;
import frc.robot.util.Coords;

import java.util.function.Supplier;
import simulationlib.shuffle.MultiType;
import simulationlib.shuffle.PrefixedConcurrentMap;
import simulationlib.shuffle.ShuffleboardHelpers;
import simulationlib.shuffle.SupplierMapFactory;

/**
 * RobotContainer.
 */
public class RobotContainer {
    private final AppliedController m_driveController = new AppliedController(
            OperatorConstants.driveControllerPort);
    private final AppliedController m_armController = new AppliedController(
            OperatorConstants.armControllerPort);

    private SwerveDriveSystem m_swerveDrive = SwerveDriveSystemSim
            .createSwerveDriveSystemInstance(m_driveController);
    private VisionSystem m_visionSystem = new VisionSystem();

    private ShooterSystem m_shooterSystem = new ShooterSystem();
    private ArmSystem m_armSystem = new ArmSystem(m_armController);
    private IntakeSystem m_intakeSystem = new IntakeSystem();
    private HookSystem m_hookSystem = new HookSystem(m_armController);
    private LEDSystem m_ledSystem = new LEDSystem(m_intakeSystem);

    SendableChooser<String> m_autoChooser = new SendableChooser<>();

    private PopulateSimShuffleboard m_shuffleboardManager = null;

    public RobotContainer() {
        m_autoChooser.setDefaultOption("3 Note Auto High", "MID-TOP 3 NOTE");
        m_autoChooser.addOption("3 Note Auto Low", "MID-BOTTOM 3 NOTE");
        m_autoChooser.addOption("Low Score Escape", "BOTTOM LEAVE 1 NOTE");
        m_autoChooser.addOption("High Score Escape", "TOP LEAVE 1 NOTE");
        m_autoChooser.addOption("One Note Stage Auto (NO MOVEMENT)", "Stage Auto Stay");
        m_autoChooser.addOption("One Note Amp Auto (NO MOVEMENT)", "Amp Auto Stay");

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
        if (RobotBase.isSimulation()) {
            initSimShuffleboard();
        }
    }

    private void initSimShuffleboard() {
        // Now that all subsystems are created, print out the list of properties
        // available for display in Shuffleboard.
        printAvailableDashboardProperties();

        m_shuffleboardManager = new PopulateSimShuffleboard(
                new ShuffleboardHelpers(SupplierMapFactory.getGlobalInstance()),
                new DefaultSimLayout(),
                Shuffleboard.getTab("Simulation"));
    }

    private void printAvailableDashboardProperties() {
        PrefixedConcurrentMap<Supplier<MultiType>> globalMap = SupplierMapFactory
                .getGlobalInstance();

        globalMap.prettyPrint();
    }

    public void updateSimShuffleboard() {
        if (m_shuffleboardManager != null) {
            m_shuffleboardManager.updateDashOnRobotPeriodic();
        }
    }

    public void scheduleAutonomousCommand() {

        // Choose which Field relative to use
        // use a sendable chooser for which gyro to reset to

        // NOTE Field relative is dependent both on which alliance you are on, but also what part of
        // the subwoofer you are on.

        // m_swerveDrive.resetGyroFieldRelativeBlueMid();
        String autoName = m_autoChooser.getSelected();
        if (autoName.equals("BOTTOM LEAVE 1 NOTE")) {
            m_swerveDrive.resetGyroFieldRelativeBlueBottom();
        }
        else if (autoName.equals("TOP LEAVE 1 NOTE")) {
            m_swerveDrive.resetGyroFieldRelativeBlueTop();
        }
        else {
            if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                m_swerveDrive.resetGyroFieldRelativeAutoRed();
            }
            else {
                m_swerveDrive.resetGyroFieldRelativeAuto();
            }
        }

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
                        Constants.SwerveSystemConstants.frameDistanceToModulesMeters,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red),
                m_swerveDrive);

        Command auto = new PathPlannerAuto(autoName);
        auto.schedule();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Arm").addDouble(
                "Angle to Shoot",
                () -> m_armSystem.getShootingAngle(m_visionSystem.getDistanceMetersY())
                        + ShooterConstants.shootOffsetLimeLight);

        if (m_shuffleboardManager != null) {
            m_shuffleboardManager.addShuffleboardWidgets();
            m_shuffleboardManager.addMacros(m_armSystem);
        }
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {

        // ARM CONTROLLER BINDINGS

        // Push note piece back on start up. May not need to happen when reflectometer is used.
        // Was 0.2, revert back if it does not work (I THINK I COULD PUT IT
        // TO .01 BECAUSE THE MOTORS STOP RUNNING WHICH MEANS IT TAKES TIME
        // TO REV UP ANYWAYS)
        double waitTime = 0.1;

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
                new AmpLightCommand(m_ledSystem));

        new Trigger(() -> m_driveController.getYButton()).onTrue(
                // RESETS LED JUST IN CASE THE CODE IS NOT RIGHT
                new LEDResetCommand(m_ledSystem));

        new Trigger(() -> m_armController.povRight(new EventLoop()).getAsBoolean()).onTrue(
                new PullBackCommand(m_intakeSystem)
                        .andThen(new WaitCommand(waitTime))
                        .andThen(
                                new RevCommandAmp(m_intakeSystem, m_shooterSystem, m_armController,
                                        0.2)));

        // Auto-align
        // new Trigger(() -> m_driveController.getAButton()).onTrue(
        // new VisionAutoAlignCommand(m_swerveDrive, m_visionSystem));

    }

    public void toAuto() {
        m_swerveDrive.toAuto();
    }

    public void toTeleop() {
        m_swerveDrive.toTeleop();
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
