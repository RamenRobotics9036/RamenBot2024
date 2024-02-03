package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.VisionAutoAlignCommand;
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

    private final ShooterSystem m_shooter = new ShooterSystem();
    private final ArmSystem m_ArmAndIntakeSystem = new ArmSystem(m_armController);

    public RobotContainer() {
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {
        new Trigger(() -> m_driveController.getYButton())
                .onTrue(new VisionAutoAlignCommand(m_swerveDrive, m_visionSystem));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_shooter.stopSystem();
        m_ArmAndIntakeSystem.stopSystem();
        m_visionSystem.stopSystem();
    }
}
