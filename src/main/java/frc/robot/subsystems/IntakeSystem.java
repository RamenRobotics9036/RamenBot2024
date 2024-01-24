package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Stop the intake system.
 */
public class IntakeSystem extends SubsystemBase {
    public final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID,
            MotorType.kBrushless);
    public DigitalInput refelectometer = new DigitalInput(IntakeConstants.reflectChannel);

    public IntakeSystem() {
        initShuffleBoard();

    }

    public double getIntakeSpeed() {
        return m_intakeMotor.get();
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public boolean getReflectometer() {
        return refelectometer.get();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake").add("Intake Speed: ", getIntakeSpeed());
    }

    @Override
    public void periodic() {
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        m_intakeMotor.stopMotor();
    }
}
