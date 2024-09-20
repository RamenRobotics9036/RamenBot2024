package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandsConstants.VisionAutoAlignConstants;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;

public class VisionAutoAlignCommand extends Command {
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
    private boolean m_translate = false;

    public VisionAutoAlignCommand(SwerveDriveSystem swerveDrive, VisionSystem visionSystem) {
        this(swerveDrive, visionSystem, VisionAutoAlignConstants.targetDistanceMeters, false);
    }

    public VisionAutoAlignCommand(
            SwerveDriveSystem swerveDrive,
            VisionSystem visionSystem,
            double targetDistanceMeters,
            boolean useTranslation) {
        m_targetDistanceMeters = targetDistanceMeters;
        m_swerveDrive = swerveDrive;
        m_visionSystem = visionSystem;
        addRequirements(m_swerveDrive, m_visionSystem);
        m_translate = useTranslation;
        m_translationXpid.setTolerance(
                VisionAutoAlignConstants.errorMarginDistanceX,
                VisionAutoAlignConstants.errorMarginVelocityX);
        m_translationYpid.setTolerance(
                VisionAutoAlignConstants.errorMarginDistanceY,
                VisionAutoAlignConstants.errorMarginVelocityY);
        m_rotationPid.setTolerance(
                VisionAutoAlignConstants.errorMarginRot,
                VisionAutoAlignConstants.errorMarginVelocityRot);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        double xspeed = 0, yspeed = 0;
        if (m_translate) {
            xspeed = m_translationXpid.calculate(m_visionSystem.getDistanceMetersX(), 0);
            yspeed = m_translationYpid
                    .calculate(m_visionSystem.getDistanceMetersY(), m_targetDistanceMeters);
        }
        double rotSpeed = m_rotationPid.calculate(m_visionSystem.getX(), 0);
        m_swerveDrive.drive(yspeed, -xspeed, rotSpeed, false);
    }

    @Override
    public boolean isFinished() {
        if (!m_visionSystem.isDetected()) {
            return true;
        }
        if (m_timer.get() >= VisionAutoAlignConstants.timeLimit) {
            return true;
        }
        if (m_rotationPid.atSetpoint()
                && (!m_translate
                        || (m_translationXpid.atSetpoint() && m_translationYpid.atSetpoint()))) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
        m_visionSystem.stopSystem();
    }
}
