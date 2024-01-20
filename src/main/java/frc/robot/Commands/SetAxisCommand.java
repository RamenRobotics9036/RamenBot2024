package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants.SetAxisConstants;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.Coords;

/**
 * SetAxisCommand.
 */
public class SetAxisCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private Timer m_timer;
    private Coords m_coordinates;
    private PIDController m_translationXpid = new PIDController(SetAxisConstants.translationPID_P,
            SetAxisConstants.translationPid_I, SetAxisConstants.translationPID_D);
    private PIDController m_translationYpid = new PIDController(SetAxisConstants.translationPID_P,
            SetAxisConstants.translationPid_I, SetAxisConstants.translationPID_D);
    private PIDController m_rotationPid = new PIDController(SetAxisConstants.rotationPID_P,
            SetAxisConstants.rotationPID_I, SetAxisConstants.rotationPID_D);

    /**
     * Constructor.
     */
    public SetAxisCommand(Coords coordinates, SwerveDriveSystem swerveDrive) {

        m_swerveDrive = swerveDrive;
        m_coordinates = coordinates;
        m_timer = new Timer();
        addRequirements(m_swerveDrive);

        m_rotationPid.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void initialize() {
        m_timer.start();

    }

    @Override
    public void execute() {

        double xspeed = m_translationXpid.calculate(m_swerveDrive.getxPosition(),
                m_coordinates.getX());
        double yspeed = m_translationYpid.calculate(m_swerveDrive.getyPosition(),
                m_coordinates.getY());
        // xSpeed=MathUtil.clamp(xSpeed, -SetAxisConstants.percentPower,
        // SetAxisConstants.percentPower);
        // ySpeed=MathUtil.clamp(ySpeed, -SetAxisConstants.percentPower,
        // SetAxisConstants.percentPower);
        double rotSpeed = m_rotationPid.calculate(m_swerveDrive.getAnglePositionAbsolute(),
                m_coordinates.getRotation());
        // rotSpeed=MathUtil.clamp(rotSpeed,-SetAxisConstants.percentPower,
        // SetAxisConstants.percentPower);
        // double xSpeed=m_coordinates.getX()-m_swerveDrive.getXPosition();
        // double ySpeed=m_coordinates.getY()-m_swerveDrive.getYPosition();
        // double
        // rotSpeed=m_coordinates.getRotation()-m_swerveDrive.getAnglePositionAbsoluteRadians();

        xspeed = MathUtil
                .clamp(xspeed, -SetAxisConstants.percentPower, SetAxisConstants.percentPower);
        yspeed = MathUtil
                .clamp(yspeed, -SetAxisConstants.percentPower, SetAxisConstants.percentPower);
        rotSpeed = MathUtil
                .clamp(rotSpeed, -SetAxisConstants.percentPower, SetAxisConstants.percentPower);

        SmartDashboard.putNumber("X Speed", xspeed);
        SmartDashboard.putNumber("Y Speed", yspeed);
        SmartDashboard.putNumber("Rot Speed", rotSpeed);

        m_swerveDrive.drive(-xspeed, -yspeed, -rotSpeed);
    }

    @Override
    public boolean isFinished() {
        // $TODO - Can this be made more readable by breaking it up?
        if (MathUtil.applyDeadband(m_swerveDrive.getxPosition() - m_coordinates.getX(),
                SetAxisConstants.errorMarginXY) == 0
                && MathUtil.applyDeadband(m_swerveDrive.getyPosition() - m_coordinates.getY(),
                        SetAxisConstants.errorMarginXY) == 0
                && (MathUtil.applyDeadband(m_swerveDrive.getAnglePositionAbsoluteRadians()
                        - m_coordinates.getRotation(), SetAxisConstants.errorMarginRot) == 0
                        || MathUtil.applyDeadband(
                                Math.PI * 2 - m_swerveDrive.getAnglePositionAbsolute(),
                                SetAxisConstants.errorMarginRot) <= m_coordinates.getRotation())) {
            return true;
        }
        if (m_timer.get() >= SetAxisConstants.timeLimit) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
