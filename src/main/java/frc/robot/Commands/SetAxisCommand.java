package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Constants.CommandsConstants.SetAxisConstants;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.Coords;


public class SetAxisCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private Timer m_timer;
    private Coords m_coordinates;
    private PIDController m_translationXPID = new PIDController
    (SetAxisConstants.translationPID_P, SetAxisConstants.translationPID_I, SetAxisConstants.translationPID_D);
      private PIDController m_translationYPID = new PIDController
    (SetAxisConstants.translationPID_P, SetAxisConstants.translationPID_I, SetAxisConstants.translationPID_D);
       private PIDController m_rotationYPID = new PIDController
    (SetAxisConstants.rotationPID_P, SetAxisConstants.rotationPID_I, SetAxisConstants.rotationPID_D);
    public SetAxisCommand(Coords coordinates,  SwerveDriveSystem swerveDrive) {
      
        m_swerveDrive = swerveDrive;
        m_coordinates = coordinates;
        m_timer = new Timer();
    }

    
    

    @Override
    public void initialize() {
        m_timer.start();
         
    }

    @Override
    public void execute() {
         {
        
            double xSpeed = m_translationXPID.calculate(m_swerveDrive.getXPosition(), m_coordinates.getX());
            double ySpeed = m_translationYPID.calculate(m_swerveDrive.getYPosition(), m_coordinates.getY());
            xSpeed=MathUtil.clamp(xSpeed, -SetAxisConstants.percentPower, SetAxisConstants.percentPower);
            ySpeed=MathUtil.clamp(ySpeed, -SetAxisConstants.percentPower, SetAxisConstants.percentPower);
            double rotSpeed = m_rotationYPID.calculate(m_swerveDrive.getAnglePosition(),m_coordinates.getRotation());
            rotSpeed=MathUtil.clamp(rotSpeed,-SetAxisConstants.percentPower, SetAxisConstants.percentPower);
            m_swerveDrive.drive(xSpeed, ySpeed, SetAxisConstants.percentPower);
            
        }
      
    }
        
        
    

    @Override
    public boolean isFinished() {
        if(MathUtil.applyDeadband(m_swerveDrive.getXPosition(), m_coordinates.getX()) <= CommandsConstants.SetAxisConstants.errorMargin && 
        MathUtil.applyDeadband(m_swerveDrive.getYPosition(), m_coordinates.getY()) <= CommandsConstants.SetAxisConstants.errorMargin) {
            return true;
        }
        if(m_timer.get() >= SetAxisConstants.timeLimit) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}

