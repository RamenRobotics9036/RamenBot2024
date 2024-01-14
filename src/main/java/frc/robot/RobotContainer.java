package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetAxisCommand;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;
import frc.robot.util.Coords;

/**
 * RobotContainer.
 */
public class RobotContainer {
    private AppliedController m_driveController = new AppliedController(
            OperatorConstants.driveControllerPort);
    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);

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
    }
}
