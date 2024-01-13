package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;
import frc.robot.Util.Coords;


public class SetAxisCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private AppliedController m_controller;
    private Coords m_coordinates;
    
    public SetAxisCommand(Coords coordinates, AppliedController controller, SwerveDriveSystem swerveDrive) {
        m_controller = controller;
        m_swerveDrive = swerveDrive;
        m_coordinates = coordinates;
        
    }

    
    

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
         {
            m_swerveDrive.getAnglePosition();
            
        }
      
    }
        
        
    

    @Override
    public boolean isFinished() {
        if(MathUtil.applyDeadband(m_swerveDrive.getXPosition(), m_coordinates.getX()) <= CommandsConstants.errorMargin && 
        MathUtil.applyDeadband(m_swerveDrive.getYPosition(), m_coordinates.getY()) <= CommandsConstants.errorMargin) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}

