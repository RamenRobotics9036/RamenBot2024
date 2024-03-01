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

public class LEDSystem extends SubsystemBase {

    // I WANT TO USE THE TOP INTAKE MOTOR BECAUSE THAT WILL ONLY CHANGE IF IT ACTUALLY GRABS A PIECE

    // THE BOTTOM NOTE MIGHT SLOW DOWN BECAUSE IT HITS THE NOTE, BUT IT MAY NOT ACTUALLY INTAKE IT
    // private final CANSparkMax intakeMotor = new CANSparkMax(
    // IntakeConstants.intakeMotorRightID,
    // MotorType.kBrushless);
    // private RelativeEncoder m_encoder = intakeMotor.getEncoder();

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

        // intakeMotor.restoreFactoryDefaults();
        // intakeMotor.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        // initShuffleBoard();

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

    /*
     * public void initShuffleBoard() {
     * Shuffleboard.getTab("Intake")
     * .add("Current Output: ", intakeMotor.getOutputCurrent());
     * Shuffleboard.getTab("Intake")
     * .add("Current Velocity: ", m_encoder.getVelocity());
     * 
     * }
     */

    // IF THIS DOES NOT WORK, ADD A VELOCITY METHOD

    @Override
    public void periodic() {
        // if (intakeMotor.getOutputCurrent() > NORMALCURRENT) {
        // // SET LED Lights to blah blah
        // }
        for (var i = 0; i < m_LEDBuffer.getLength() / 2; i++) {
           
                m_LEDBuffer.setRGB(i, m_ledG, m_ledR, m_ledB);
                m_LEDBuffer.setRGB(m_LEDBuffer.getLength() - i - 1, m_ledG, m_ledR, m_ledB);
        
        }
        m_ledR = 100;
        m_ledG = 255;
        m_ledB = 100;
        System.out.println("This is not working");

        m_LEDLight.setData(m_LEDBuffer);
        m_LEDLight.start();
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {

    }
}
