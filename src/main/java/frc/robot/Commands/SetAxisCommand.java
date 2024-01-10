package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;
import frc.robot.Util.Coords;


public class SetAxisCommand extends CommandBase {
    private SwerveDriveSystem swerveDrive;
    private AppliedController controller;
    
    
    public SetAxisCommand(double value, AppliedController m_controller, SwerveDriveSystem m_swerveDrive) {
        m_controller = controller;
        swerveDrive = m_swerveDrive;
        
    }

    
    

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
         {
            swerveDrive.getAnglePosition();
            
           
        }
      
        }
        
        
    

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
       // m_swerveDrive.stopSystem();
    }
}

