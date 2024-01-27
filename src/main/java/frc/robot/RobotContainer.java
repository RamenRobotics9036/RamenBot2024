package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.WriteYCommand;
import frc.robot.subsystems.ArmAndIntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;

/**
 * RobotContainer.
 */
public class RobotContainer {
    private final AppliedController m_driveController = new AppliedController(
            OperatorConstants.driveControllerPort);

    private final SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);
    private final ShooterSystem m_shooter = new ShooterSystem();
    private final ArmAndIntakeSystem m_ArmAndIntakeSystem = new ArmAndIntakeSystem();

    public RobotContainer() {
    }

    /**
     * This is the single place that joystick triggers/buttons are bound to specific commands.
     */
    public void bindCommands() {
        double speed = 0.2;
        double maxTime = 2;

        new Trigger(() -> m_driveController.getAButton())
                .onTrue(new WriteYCommand(m_swerveDrive, speed, maxTime, 0));

        new Trigger(() -> m_driveController.getBButton())
                .onTrue(new WriteYCommand(m_swerveDrive, speed, maxTime, 0));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_shooter.stopSystem();
        m_ArmAndIntakeSystem.stopSystem();
    }
}
