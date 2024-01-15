package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetAxisCommand;
import frc.robot.subsystems.ArmAndIntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;
import frc.robot.util.Coords;

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
        new Trigger(() -> m_driveController.getAButton())
                .onTrue(new SetAxisCommand(new Coords(0, 0, 0), m_swerveDrive));

        new Trigger(() -> m_driveController.getBButton()).onTrue(new SetAxisCommand(
                new Coords(m_swerveDrive.getxPosition() + 2, m_swerveDrive.getyPosition(),
                        m_swerveDrive.getAnglePositionAbsoluteRadians()),
                m_swerveDrive));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
        m_shooter.stopSystem();
        m_ArmAndIntakeSystem.stopSystem();
    }
}
