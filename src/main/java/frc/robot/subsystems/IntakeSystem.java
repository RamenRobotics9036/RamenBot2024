package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.util.AppliedController;

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

    enum RumblingState {
        NoCapture, Rumbling, HasRumbled
    };

    private RumblingState m_rumblingState = RumblingState.NoCapture;
    private Timer m_rumblingTimer = new Timer();
    private AppliedController m_Controller;

    public IntakeSystem(LEDSystem ledSystem, AppliedController controller) {
        m_Controller = controller;
        m_intakeMotorFollower.restoreFactoryDefaults();
        m_intakeMotorLeader.restoreFactoryDefaults();

        m_intakeMotorLeader.setIdleMode(IdleMode.kBrake);
        m_intakeMotorFollower.setIdleMode(IdleMode.kBrake);

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
                .addBoolean("Beam Back", () -> m_LedSystem.getBeamBreakShooter());
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
        if (RobotState.isAutonomous()) {
            boolean shotNote = true; // $TODO - This variable is never set to false! Needs logic
                                     // cleanup

            if (useBeamBreak || RobotState.isAutonomous()) {
                if (!m_LedSystem.getBeamBreakIntake() || !m_LedSystem.getBeamBreakShooter()) {
                    ShooterConstants.shouldCharge = true;
                }

                // No note
                if (m_LedSystem.getBeamBreakShooter() && m_LedSystem.getBeamBreakIntake()) {
                    setIntakeSpeed(IntakeConstants.intakeSpeed);
                    shotNote = true;
                }
                // Has note, but needs to be pulled back
                else if (!m_LedSystem.getBeamBreakShooter() && !m_LedSystem.getBeamBreakIntake()) {
                    // $TODO - Is this a bug? If we know we need to pull-back the note, then why is
                    // ShooterConstants.shouldCharge set to true? Wouldn't that potentially
                    // cause a weak shot? Or another way to put it - why are we needing
                    // to pull back the note here?
                    ShooterConstants.shouldCharge = true;
                    if (shotNote) {
                        setIntakeSpeed(IntakeConstants.pullBackSpeed);
                    }
                    // Has note and is pulled back
                    // $TODO - This is a bug? There's no possible way for getBeamBreakPullBack to
                    // become
                    // TRUE since we're in the else if block that checks for it being FALSE.
                    //
                    // Some piece of code somowhere may be stopping the intake (by setting it to
                    // speed
                    // 0), but its clearly not this code...
                    if (m_LedSystem.getBeamBreakShooter() && !m_LedSystem.getBeamBreakIntake()
                            && shotNote) {
                        setIntakeSpeed(0);
                    }
                }
            }
        }
        else if (RobotState.isTeleop() && useBeamBreak) {
            if (useBeamBreak || RobotState.isAutonomous()) {
                // Note is detected by either beam breaks
                if (!m_LedSystem.getBeamBreakIntake() || !m_LedSystem.getBeamBreakShooter()) {
                    ShooterConstants.shouldCharge = true;
                }

                // No note
                if (m_LedSystem.getBeamBreakShooter() && m_LedSystem.getBeamBreakIntake()) {
                    setIntakeSpeed(IntakeConstants.intakeSpeed);
                    ShooterConstants.shouldCharge = false;
                }

                // Note blocking both beam breaks
                if (!m_LedSystem.getBeamBreakShooter() && !m_LedSystem.getBeamBreakIntake()) {
                    // $TODO - Is this a bug? If we know we need to pull-back the note, then why is
                    // ShooterConstants.shouldCharge set to true? Wouldn't that potentially
                    // cause a weak shot? Or another way to put it - why are we needing
                    // to pull back the note here?
                    setIntakeSpeed(0);
                }
            }

            if (useBeamBreak) {
                boolean captured = !m_LedSystem.getBeamBreakShooter()
                        || !m_LedSystem.getBeamBreakIntake();
                if (m_rumblingState == RumblingState.NoCapture) {
                    if (captured) {
                        m_rumblingTimer.restart();
                        m_rumblingState = RumblingState.Rumbling;
                        m_Controller.setRumble(
                                RumbleType.kBothRumble,
                                0.3);
                    }
                }
                else if (m_rumblingState == RumblingState.Rumbling) {
                    if (!captured || m_rumblingTimer.get() > 0.5) {
                        m_Controller.setRumble(RumbleType.kBothRumble, 0.0);
                        m_rumblingState = RumblingState.HasRumbled;
                    }
                    else {
                        m_Controller.setRumble(
                                RumbleType.kBothRumble,
                                0.3);
                    }
                }
                else if (m_rumblingState == RumblingState.HasRumbled) {
                    if (!captured) {
                        m_rumblingState = RumblingState.NoCapture;
                    }
                }
            }
        }
        else {
            // $TODO - Note that right now, this code is never used, and the block of code above
            // is used for tele and auto always, since useBeamBreak is never set to False currently.
            // It's *almost* set to false in PullIntakeCommand, but that command is never used.
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

    // $TODO - Note that nobody calls this method.
    public void stopBeamBreakSystem() {
        useBeamBreak = false;
    }
}
