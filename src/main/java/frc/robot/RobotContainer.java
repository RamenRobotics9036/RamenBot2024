package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.IntakeRevCommand;
import frc.robot.commands.SetIntakeSpeedCommand;
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
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {
        // Push note piece back on start up. May not need to happen when reflectometer is used.
        double pullBackNoteTime = 0.2;
        double pullBackNoteSpeed = 0.2;
        new Trigger(() -> m_armController.getAButton()).onTrue(
                new SetIntakeSpeedCommand(m_intakeSystem, pullBackNoteTime, pullBackNoteSpeed)
                        .andThen(
                                new IntakeRevCommand(m_intakeSystem, m_shooterSystem,
                                        m_armController)));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_visionSystem.stopSystem();

        m_shooterSystem.stopSystem();
        m_armSystem.stopSystem();
        m_intakeSystem.stopSystem();
    }
}
