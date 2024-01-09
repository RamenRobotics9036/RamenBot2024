package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;

public class RobotContainer {
    private AppliedController m_driveController = new AppliedController(OperatorConstants.driveControllerPort);
    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);

    public RobotContainer() {
        NamedCommands.registerCommand("Instant Command", new InstantCommand());
    }

    public void scheduleAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        new FollowPathHolonomic(
            path,
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::getSpeeds,
            m_swerveDrive::driveFromChassisSpeeds,
            new HolonomicPathFollowerConfig(
                SwerveSystemConstants.maxSpeedMetersPerSecond,
                m_swerveDrive.getDriveBaseRadius(),
                new ReplanningConfig()
            ),
            () -> false,
            m_swerveDrive
        ).schedule();
    }

    public void bindCommands() {
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
    }
}