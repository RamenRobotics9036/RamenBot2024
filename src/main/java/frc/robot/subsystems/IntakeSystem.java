package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeDefaultCommand;

/**
 * Stop the intake system.
 */
public class IntakeSystem extends SubsystemBase {
    private final CANSparkMax m_IntakeMotorFollower = new CANSparkMax(
            IntakeConstants.intakeMotorLeftID,
            MotorType.kBrushless);
    private final CANSparkMax m_intakeMotorLeader = new CANSparkMax(
            IntakeConstants.intakeMotorRightID,
            MotorType.kBrushless);
    private DigitalInput refelectometer = new DigitalInput(IntakeConstants.reflectChannel);
    private double maxOutputPercent = IntakeConstants.maxOutputPercent;

    private AbsoluteEncoder m_leaderEncoder = m_intakeMotorLeader.getAbsoluteEncoder(Type.kDutyCycle);
    private AbsoluteEncoder m_followerEncoder = m_IntakeMotorFollower.getAbsoluteEncoder(Type.kDutyCycle);

    private boolean[] m_status = new boolean[2];

    public IntakeSystem() {
        m_IntakeMotorFollower.restoreFactoryDefaults();
        m_intakeMotorLeader.restoreFactoryDefaults();
        m_intakeMotorLeader.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        // This motor has a lot of friction in the mechanical system. Set this to the
        // constant value
        // when this issue is fixed, increasing the current limit is a workaround for
        // this issue.
        m_IntakeMotorFollower.setSmartCurrentLimit(40);
        initShuffleBoard();
        m_IntakeMotorFollower.setInverted(true);
        m_intakeMotorLeader.setInverted(true);
        m_IntakeMotorFollower.follow(m_intakeMotorLeader);
        setDefaultCommand(new IntakeDefaultCommand(this));
    }

    public double getIntakeSpeed() {
        return m_intakeMotorLeader.get();
    }

    public void setIntakeSpeed(double speed) {
        speed = MathUtil.clamp(speed, -maxOutputPercent, maxOutputPercent);
        m_intakeMotorLeader.set(speed);
    }

    public boolean getReflectometer() {
        return refelectometer.get();
    }

    public double getLeaderPosition() {
        return m_leaderEncoder.getPosition();
    }

    public double getFollowerPosition() {
        return m_followerEncoder.getPosition();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake").add("Intake Speed: ", getIntakeSpeed());
        Shuffleboard.getTab("Intake").add("Current Command", this);

        Shuffleboard.getTab("IntakeTest").addBoolean("Leader", () -> m_status[0]);
        Shuffleboard.getTab("IntakeTest").addBoolean("Follower", () -> m_status[1]);
    }

    @Override
    public void periodic() {
    }

    public void setStatus(boolean[] status) {
        m_status = status;
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        m_intakeMotorLeader.stopMotor();
    }
}
