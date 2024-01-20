package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants.VisionAutoAlignConstants;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;

public class VisionAutoAlignCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private Timer m_timer;
    private PIDController m_translationXpid = new PIDController(VisionAutoAlignConstants.translationPID_P,
            VisionAutoAlignConstants.translationPid_I, VisionAutoAlignConstants.translationPID_D);
    private PIDController m_translationYpid = new PIDController(VisionAutoAlignConstants.translationPID_P,
            VisionAutoAlignConstants.translationPid_I, VisionAutoAlignConstants.translationPID_D);
    private PIDController m_rotationPid = new PIDController(VisionAutoAlignConstants.rotationPID_P,
            VisionAutoAlignConstants.rotationPID_I, VisionAutoAlignConstants.rotationPID_D);

    private VisionSystem m_VisionSystem = new VisionSystem();

    public VisionAutoAlignCommand(SwerveDriveSystem swerveDrive) {
        m_swerveDrive = swerveDrive;
        m_timer = new Timer();
        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() { // if to left then turn right if to right then turn left else dont care
        if (m_VisionSystem.isDetected()) {
            double xAngle = m_VisionSystem.getX();
            double rotation = -xAngle / 90;

            if (xAngle < 0) { 
                m_swerveDrive.drive(0, 0, rotation);
            }
            else if (xAngle > 0) {
                m_swerveDrive.drive(0, 0, rotation);
            } 
        }
          
    }
    @Override
    public boolean isFinished() {
        if (m_timer.get() >= VisionAutoAlignConstants.timeLimit) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
