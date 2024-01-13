package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SetAxisCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;
import frc.robot.Util.Coords;

public class RobotContainer {
    private AppliedController m_driveController = new AppliedController(OperatorConstants.driveControllerPort);
    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);

    public RobotContainer() {
    }

    public void bindCommands() {
        new Trigger(() -> m_driveController.getAButton()).onTrue(new SetAxisCommand(new Coords(0, 0, 0), m_swerveDrive));
        new Trigger(() -> m_driveController.getBButton()).onTrue(new SetAxisCommand(new Coords(m_swerveDrive.getXPosition() + 2, m_swerveDrive.getYPosition(), m_swerveDrive.getAnglePositionAbsoluteRadians()), m_swerveDrive));
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
    }
}