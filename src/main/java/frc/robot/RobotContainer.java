package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;

public class RobotContainer {
    private AppliedController m_driveController = new AppliedController(OperatorConstants.driveControllerPort);
    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);

    public RobotContainer() {
    }

    public void bindCommands() {
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
    }
}