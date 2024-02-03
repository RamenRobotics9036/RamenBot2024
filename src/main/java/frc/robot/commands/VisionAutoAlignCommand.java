package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants.VisionAutoAlignConstants;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;

public class VisionAutoAlignCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private Timer m_timer;
    private PIDController m_translationXpid = new PIDController(
            VisionAutoAlignConstants.translationXPID_P, VisionAutoAlignConstants.translationXPID_I,
            VisionAutoAlignConstants.translationXPID_D);
    private PIDController m_translationYpid = new PIDController(
            VisionAutoAlignConstants.translationYPID_P, VisionAutoAlignConstants.translationYPID_I,
            VisionAutoAlignConstants.translationYPID_D);
    private PIDController m_rotationPid = new PIDController(VisionAutoAlignConstants.rotationPID_P,
            VisionAutoAlignConstants.rotationPID_I, VisionAutoAlignConstants.rotationPID_D);

    private VisionSystem m_visionSystem;
    private double m_targetDistanceMeters = VisionAutoAlignConstants.targetDistanceMeters;

    public VisionAutoAlignCommand(SwerveDriveSystem swerveDrive, VisionSystem visionSystem) {
        m_swerveDrive = swerveDrive;
        m_visionSystem = visionSystem;
        m_timer = new Timer();
        addRequirements(m_swerveDrive, m_visionSystem);

        m_translationXpid.setTolerance(VisionAutoAlignConstants.errorMarginDistanceX);
        m_translationYpid.setTolerance(VisionAutoAlignConstants.errorMarginDistanceY);
        m_rotationPid.setTolerance(VisionAutoAlignConstants.errorMarginRot);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        double xspeed = m_translationXpid.calculate(m_visionSystem.getDistanceMetersX(), 0);
        double yspeed = m_translationYpid.calculate(
                m_visionSystem.getDistanceMetersY(),
                m_targetDistanceMeters);
        double rotSpeed = m_rotationPid.calculate(m_visionSystem.getX(), 0);

        m_swerveDrive.drive(-xspeed, -yspeed, -rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        if (!m_visionSystem.isDetected()) {
            return true;
        }
        if (m_timer.get() >= VisionAutoAlignConstants.timeLimit) {
            return true;
        }
        if (MathUtil.applyDeadband(
                m_visionSystem.getDistanceMetersY() - m_targetDistanceMeters,
                VisionAutoAlignConstants.errorMarginDistanceY) == 0
                && MathUtil.applyDeadband(
                        m_visionSystem.getDistanceMetersX(),
                        VisionAutoAlignConstants.errorMarginDistanceX) == 0
                && MathUtil.applyDeadband(
                        m_visionSystem.getX(),
                        VisionAutoAlignConstants.errorMarginRot) == 0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
