package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSystem extends SubsystemBase {

    // I WANT TO USE THE TOP INTAKE MOTOR BECAUSE THAT WILL ONLY CHANGE IF IT ACTUALLY GRABS A PIECE

    // THE BOTTOM NOTE MIGHT SLOW DOWN BECAUSE IT HITS THE NOTE, BUT IT MAY NOT ACTUALLY INTAKE IT
    private final CANSparkMax intakeMotor = new CANSparkMax(
            IntakeConstants.intakeMotorRightID,
            MotorType.kBrushless);
    private RelativeEncoder m_encoder = intakeMotor.getEncoder();

    // ARBITRARY NUMBER
    private double NORMALCURRENT = Double.MAX_VALUE;

    private AddressableLED m_LEDLight = new AddressableLED(
            Constants.OperatorConstants.kLEDLightsChannel);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(
            Constants.OperatorConstants.kLEDLightsLength);

    private int m_ledLoop;
    private int m_ledR;
    private int m_ledG;
    private int m_ledB;
    private int m_ledHue;

    public LEDSystem() {

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        initShuffleBoard();

        m_ledLoop = 0;
        m_ledR = 0;
        m_ledG = 0;
        m_ledB = 0;
        m_ledHue = 0;
        m_LEDLight.setLength(m_LEDBuffer.getLength());

    }

    /**
     * Sets the LED lights to yellow.
     */
    public void setLedsYellow() {
        m_ledR = 255;
        m_ledG = 255;
        m_ledB = 0;
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake")
                .add("Current Output: ", intakeMotor.getOutputCurrent());
        Shuffleboard.getTab("Intake")
                .add("Current Velocity: ", m_encoder.getVelocity());

    }

    // IF THIS DOES NOT WORK, ADD A VELOCITY METHOD

    @Override
    public void periodic() {
        // if (intakeMotor.getOutputCurrent() > NORMALCURRENT) {
        // // SET LED Lights to blah blah
        // }
        m_ledR = 255;
        m_ledG = 255;
        m_ledB = 0;

    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        intakeMotor.stopMotor();
    }
}
