package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PresetConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.commands.ChargeShootForTimeCommand;
import frc.robot.commands.IntakeRevCommand;
import frc.robot.commands.IntakeRevCommandAuto;
import frc.robot.commands.PullBackCommand;
import frc.robot.commands.PullBackShooterCommand;
import frc.robot.commands.RevCommandAmp;
import frc.robot.commands.RotatePIDCommand;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.commands.ShootCommandTele;
import frc.robot.commands.StayCommand;
import frc.robot.commands.ShootCommands.ChargeShootCommand;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.HookSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.AppliedController;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

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
    private LEDSystem m_LEDSystem = new LEDSystem();
    private IntakeSystem m_intakeSystem = new IntakeSystem(m_LEDSystem);
    private HookSystem m_hookSystem = new HookSystem(m_armController);

    @SuppressWarnings({
            "AbbreviationAsWordInNameCheck", "MemberNameCheck"
    })

    SendableChooser<String> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        m_autoChooser.setDefaultOption("3 Note Auto High", "MID-AMP 3 NOTE");
        m_autoChooser.addOption("3 Note Auto Low", "MID-STAGE 3 NOTE");
        m_autoChooser.addOption("Low Score Escape", "STAGE SUB LEAVE 1 NOTE");
        m_autoChooser.addOption("High Score Escape", "Amp Leave 1 Note");
        m_autoChooser.addOption("One Note Stage Auto (NO MOVEMENT)", "Stage Auto Stay");
        m_autoChooser.addOption("One Note Amp Auto (NO MOVEMENT)", "Amp Auto Stay");
        m_autoChooser.addOption("4 Note Auto?????", "Potential 4 Note Auto");
        m_autoChooser.addOption("4 Note Auto Diagonal", "4 Note Diagonal");
        m_autoChooser.addOption(
                "4 Note Auto Diagonal While Moving",
                "4 Note Diagonal (SHOOT WHILE MOVING)");

        m_autoChooser.addOption(
                "Amp Sub 3 Note (Amp Note and Top Center Note)",
                "Amp Sub 3 Note (Amp Note and Top Center Note)");

        m_autoChooser.addOption("Move 2 Meters", "Move 2 Meters");

        Shuffleboard.getTab("Auto").add(m_autoChooser);
        // Shuffleboard.getTab("Swerve")
        // .addDouble("Rotation Angle", () -> m_swerveDrive.getRotation2d().getRadians());

        @SuppressWarnings("VariableDeclarationUsageDistanceCheck")
        double waitTime = 0.2;
        initShuffleBoard();

        // I will probably need to add a timer or maybe I can do that in Path Planner
        NamedCommands.registerCommand(
                "Set Arm To Ground",
                new SetArmToAngleCommand(m_armSystem, SetArmConstants.armMin));

        // THIS IS A TEST AND MIGHT BREAK THE CODE
        NamedCommands.registerCommand(
                "Raise Arm and Shoot Note WHILE MOVING",
                new ParallelCommandGroup(
                        new SetArmToAngleCommand(m_armSystem,
                                PresetConstants.speakerPresetAngleAutoRadians),
                        new RevCommandAmp(m_intakeSystem, m_shooterSystem, m_armController,
                                0.65)) // THIS TIMING WILL NEED TO BE LOOKED AT WITH SET
                                       // ARM TO ANGLE COMMAND
        );

        // THIS COULD POTENTIALLY RAISE ARM AND DO PULL BACK AT THE SAME TIME
        NamedCommands.registerCommand(
                "Raise Arm and Shoot Note",
                new ParallelDeadlineGroup(
                        new ParallelCommandGroup(
                                new SetArmToAngleCommand(m_armSystem,
                                        PresetConstants.speakerPresetAngleAutoRadians),
                                new RevCommandAmp(m_intakeSystem, m_shooterSystem, m_armController,
                                        0.65)), // THIS TIMING WILL NEED TO BE LOOKED AT WITH SET
                                                // ARM TO ANGLE COMMAND
                        new StayCommand(m_swerveDrive)));
    }

    @SuppressWarnings("AbbreviationAsWordInNameCheck")
    private JSONObject getAutoJSON(String autoName) {
        JSONObject json;
        try (BufferedReader br = new BufferedReader(
                new FileReader(
                        new File(
                                Filesystem.getDeployDirectory(),
                                "pathplanner/autos/" + autoName + ".auto")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            json = (JSONObject) new JSONParser().parse(fileContent);
        }
        catch (Exception e) {
            throw new RuntimeException(e.getMessage());
        }
        return json;
    }

    public void scheduleAutonomousCommand() {

        // Choose which Field relative to use
        // use a sendable chooser for which gyro to reset to

        // NOTE Field relative is dependent both on which alliance you are on, but also what part of
        // the subwoofer you are on.

        // m_swerveDrive.resetGyroFieldRelativeBlueMid();
        String autoName = m_autoChooser.getSelected();
        JSONObject autoSettings = getAutoJSON(autoName);
        Pose2d startPose = AutoBuilder
                .getStartingPoseFromJson((JSONObject) autoSettings.get("startingPose"));

        // m_swerveDrive.resetPose(startPose);
        double angle = startPose.getRotation().getDegrees();
        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
            angle += 180;
        }
        m_swerveDrive.resetGyroToAngle(Rotation2d.fromDegrees(angle).getDegrees());

        AutoBuilder.configureHolonomic(
                m_swerveDrive::getPoseMeters,
                m_swerveDrive::resetPose,
                m_swerveDrive::getSpeeds,
                m_swerveDrive::driveFromChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0.1),
                        SwerveSystemConstants.maxSpeedMetersPerSecondAuto,
                        m_swerveDrive.getDriveBaseRadius(),
                        new ReplanningConfig(true, true)),
                () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red),
                m_swerveDrive);

        Command auto = new PathPlannerAuto(autoName);
        auto.schedule();
    }

    public void initShuffleBoard() {
        // Shuffleboard.getTab("Vision").addDouble(
        // "Angle to Shoot",
        // () -> m_armSystem.getShootingAngle(m_visionSystem.getSpeakerYDistance())
        // + ShooterConstants.shootOffsetLimeLight);
        Shuffleboard.getTab("Charge Shot")
                .addString(
                        "Current Shooter Command",
                        () -> (m_shooterSystem.getCurrentCommand() == null) ? "None"
                                : m_shooterSystem.getCurrentCommand().getName());
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
        double waitTime = 0.2;

        new Trigger(() -> m_armController.getAButton()).onTrue(
                new RevCommandAmp(m_intakeSystem, m_shooterSystem, m_armController,
                        0.6));

        // Amp Preset
        new Trigger(() -> m_armController.getXButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.ampPresetAngleRadians));

        // Sub-woofer Preset
        new Trigger(() -> m_armController.getYButton()).onTrue(
                new SetArmToAngleCommand(m_armSystem, PresetConstants.speakerPresetAngleRadians));

        // One robot away preset
        new Trigger(() -> m_armController.getBButton())
                .onTrue(
                        new SetArmToAngleCommand(m_armSystem,
                                PresetConstants.speakerPresetAngleRadians))
                .onFalse(
                        new SetArmToAngleCommand(m_armSystem,
                                PresetConstants.speakerPresetAngleRadians)
                                .andThen(new ShootCommandTele(m_intakeSystem, m_armController)));

        // new Trigger(() -> m_armController.getBButtonReleased()).onTrue(
        // new SetArmToAngleCommand(m_armSystem,
        // m_armSystem.getShootingAngle(m_visionSystem.getSpeakerYDistance())).andThen(
        // new ShootCommandTele(m_intakeSystem, m_armController)));

        // new Trigger(() -> m_armController.getBButtonPressed()).onTrue(
        // new SetArmToAngleCommand(m_armSystem,
        // m_armSystem.getShootingAngle(m_visionSystem.getSpeakerYDistance())));

        // 62 inches away (around podium distance) //Good radians is 4.837, but the preset is not
        // hitting the right angle (basically an offset)
        new Trigger(() -> m_armController.getStartButton())
                .onTrue(new SetArmToAngleCommand(m_armSystem, 4.824));

        // DRIVE CONTROLLER BINDINGS

        // new Trigger(() -> m_driveController.getBButton()).onTrue(
        // // AMP LIGHT COMMAND
        // new AmpLightCommand(m_LEDSystem));

        // new Trigger(() -> m_driveController.getYButton()).onTrue(
        // // RESETS LED JUST IN CASE THE CODE IS NOT RIGHT
        // new LEDResetCommand(m_LEDSystem));

        new Trigger(() -> m_driveController.getXButton()).onTrue(
                // RESETS LED JUST IN CASE THE CODE IS NOT RIGHT
                new RotatePIDCommand(m_swerveDrive, 1.5 * Math.PI));

        new Trigger(() -> m_armController.povRight(new EventLoop()).getAsBoolean()).onTrue(

                new RevCommandAmp(m_intakeSystem, m_shooterSystem, m_armController,
                        0.2));

        new Trigger(() -> m_armController.povDown(new EventLoop()).getAsBoolean())
                .onTrue(new PullBackShooterCommand(m_shooterSystem));

        new Trigger(() -> ShooterConstants.shouldCharge)
                .whileTrue(
                        new ChargeShootCommand(m_shooterSystem, m_armController));

        // new Trigger(() -> m_armController.povLeft(new EventLoop()).getAsBoolean())
        // .onTrue(); // IS IT POSSIBLE TO CALL A METHOD INSTEAD OF A COMMAND? OR WHERE ELSE DO
        // I CALL IT? (above is called bind commands, so probably not)

        // Auto-align
        // new Trigger(() -> m_driveController.getAButton()).onTrue(
        // new VisionAutoAlignCommand(m_swerveDrive, m_visionSystem));

    }

    public void toAuto() {
        ShooterConstants.shouldCharge = false;
        m_swerveDrive.toAuto();
        m_armSystem.toAuto();
    }

    public void toTeleop() {
        ShooterConstants.shouldCharge = false;
        m_swerveDrive.toTeleop();
        m_armSystem.toTeleop();
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_LEDSystem.stopSystem();

        m_shooterSystem.stopSystem();
        m_armSystem.stopSystem();
        m_intakeSystem.stopSystem();
        m_hookSystem.stopSystem();
    }
}
