package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;

public class RobotContainer {
    private AppliedController m_driveController = new AppliedController(OperatorConstants.driveControllerPort);
    private SwerveDriveSystem m_swerveDrive = new SwerveDriveSystem(m_driveController);

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Command A", Commands.print("Command A Used"));  
        configurePathPlanner();

        // autoChooser = AutoBuilder.buildAutoChooser();
        // autoTab.add("Auto Mode", autoChooser);
    }

    private void configurePathPlanner() {
        autoTab.add("Example Auto", new PathPlannerAuto("Example Auto"));
    }

    public void bindCommands() {
    }

    public void stopRobot() {
        m_swerveDrive.stopSystem();
    }
}