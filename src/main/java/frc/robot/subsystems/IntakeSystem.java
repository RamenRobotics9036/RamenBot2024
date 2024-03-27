package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeDefaultCommand;

/**
 * Stop the intake system.
 */
public class IntakeSystem extends SubsystemBase {
    private final CANSparkMax m_intakeMotorFollower = new CANSparkMax(
            IntakeConstants.intakeMotorLeftID,
            MotorType.kBrushless);
    private final CANSparkMax m_intakeMotorLeader = new CANSparkMax(
            IntakeConstants.intakeMotorRightID,
            MotorType.kBrushless);
    private DigitalInput m_refelectometer = new DigitalInput(IntakeConstants.reflectChannel);
    private RelativeEncoder m_encoder = m_intakeMotorLeader.getEncoder();
    private double m_maxOutputPercent = IntakeConstants.maxOutputPercent;

    private LEDSystem m_LedSystem;

    private boolean useBeamBreak = true;

    public IntakeSystem(LEDSystem ledSystem) {
        m_intakeMotorFollower.restoreFactoryDefaults();
        m_intakeMotorLeader.restoreFactoryDefaults();
        m_intakeMotorLeader.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        // This motor has a lot of friction in the mechanical system. Set this to the constant value
        // when this issue is fixed, increasing the current limit is a workaround for this issue.
        m_intakeMotorFollower.setSmartCurrentLimit(20);

        m_LedSystem = ledSystem;
        Shuffleboard.getTab("Charge Shot")
                .addBoolean("Should Charge", () -> ShooterConstants.shouldCharge);
        Shuffleboard.getTab("Charge Shot")
                .addBoolean("Beam Intake", () -> m_LedSystem.getBeamBreakIntake());
        Shuffleboard.getTab("Charge Shot")
                .addBoolean("Beam Back", () -> m_LedSystem.getBeamBreakPullBack());
        // initShuffleBoard();
        m_intakeMotorFollower.setInverted(true);
        m_intakeMotorLeader.setInverted(true);
        m_intakeMotorFollower.follow(m_intakeMotorLeader);
        // setDefaultCommand(new IntakeDefaultCommand(this));
    }

    public double getIntakeSpeed() {
        return m_intakeMotorLeader.get();
    }

    public double getIntakeAngle() {
        return m_encoder.getPosition();
    }

    public void setIntakeSpeed(double speed) {
        speed = MathUtil.clamp(speed, -m_maxOutputPercent, m_maxOutputPercent);
        m_intakeMotorLeader.set(speed);
    }

    public boolean getReflectometer() {
        return m_refelectometer.get();
    }

    public double getOutputCurrent() {
        return m_intakeMotorLeader.getAppliedOutput();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake").add("Intake Speed: ", getIntakeSpeed());
        Shuffleboard.getTab("Intake").addString(
                "Current Command",
                () -> (getCurrentCommand() != null) ? getCurrentCommand().getName() : "EXCEPT");

        Shuffleboard.getTab("Intake")
                .add("Current Output: ", m_intakeMotorLeader.getOutputCurrent());
        Shuffleboard.getTab("Intake")
                .add("Current Velocity: ", m_encoder.getVelocity());
    }

    @Override
    public void periodic() {

        boolean shotNote = true;

        if (useBeamBreak) {
            // No note
            if (m_LedSystem.getBeamBreakPullBack() && m_LedSystem.getBeamBreakIntake()) {
                setIntakeSpeed(IntakeConstants.intakeSpeed);
                shotNote = true;
            }
            // Has note, but needs to be pulled back
            else if (!m_LedSystem.getBeamBreakPullBack() && !m_LedSystem.getBeamBreakIntake()) {
                ShooterConstants.shouldCharge = true;
                if (shotNote) {
                    setIntakeSpeed(IntakeConstants.pullBackSpeed);
                }
                // Has note and is pulled back
                if (m_LedSystem.getBeamBreakPullBack() && !m_LedSystem.getBeamBreakIntake()
                        && shotNote) {
                    setIntakeSpeed(0);
                }
            }

        }
        else {
            setIntakeSpeed(IntakeConstants.intakeSpeed); // DAVID NEEDS TO MAKE A MANUAL PULLBACK
                                                         // COMMAND for both shooter and intake
        }
    }

    public void setOverride(boolean useSensors) {
        useBeamBreak = useSensors;
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        m_intakeMotorLeader.stopMotor();
    }

    public void stopBeamBreakSystem() {
        useBeamBreak = false;
    }
}
