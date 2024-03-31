package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.SwerveDriveSystemAbstract;

public class RotatePIDCommand extends Command {
    private SwerveDriveSystemAbstract m_swerveDrive;
    private double m_targetAngle;
    private PIDController m_pidController;

    public RotatePIDCommand(SwerveDriveSystemAbstract swerveDrive, double targetAngle) {
        m_swerveDrive = swerveDrive;
        m_targetAngle = targetAngle;
        m_pidController = new PIDController(3,
                0, 0);
        addRequirements(m_swerveDrive);

        m_pidController.setTolerance(0.007);
        // m_pidController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void initialize() {
        m_swerveDrive.resetGyroToAngle(0);
    }

    @Override
    public void execute() {
        double speed = m_pidController
                .calculate(
                        m_swerveDrive.getRotation2d().getRadians(),
                        m_targetAngle);
        // speed = MathUtil.clamp(speed, -0.01, 0.01);
        m_swerveDrive.drive(
                0,
                0,
                speed);
    }

    @Override
    public boolean isFinished() {
        if (m_pidController.atSetpoint()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
