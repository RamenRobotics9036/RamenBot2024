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
    // private final CANSparkMax intakeMotor = new CANSparkM
    // IntakeConstants.intakeMotorRight
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

    private IntakeSystem m_intakeSystem;

    public LEDSystem(IntakeSystem intakeSystem) {

        // intakeMotor.restoreFactoryDefaults();
        // intakeMotor.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        // initShuffleBoard();

        m_ledLoop = 0;
        m_ledR = 0;
        m_ledG = 0;
        m_ledB = 0;
        m_ledHue = 0;
        m_LEDLight.setLength(m_LEDBuffer.getLength());

        m_intakeSystem = intakeSystem;
    }

    /**
     * Sets the LED lights to yellow.
     */
    public void setLedsYellow() {
        m_ledR = 255;
        m_ledG = 255;
        m_ledB = 0;
    }

    public void resetLED() {
        m_ledR = 255;
        m_ledG = 0;
        m_ledB = 0;
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake")
                .add("Current Output: ", m_intakeSystem.getOutputCurrent());
    }

    // IF THIS DOES NOT WORK, ADD A VELOCITY METHOD (Look at encoder values in Intake System to see
    // if it works)

    @Override
    public void periodic() {

        // FIND WHAT THE NORMALCURRENT OF THE MOTOR RUNS AT TOMORROW
        // SEE IF THERE IS A SPIKE WHEN IT IS INTAKING A PIECE (HOLD THE PIECE SO THE INTAKE CANNOT
        // FULLY GRAB IT)

        // THINK ABOUT HOW IT WILL KNOW IF IT DOESNT HAVE A PIECE
        // for now, when it shoots, thats when it will know that it used a piece. I can check this
        // by other checking the velocity of the shoot motors or by seeing if the shoot command was
        // run

        if (m_intakeSystem.getOutputCurrent() > NORMALCURRENT) {
            // m_LEDBuffer.setHSV()
        }

        if (0 == m_ledR && 0 == m_ledG && 0 == m_ledB) {
            for (var i = 0; i < m_LEDBuffer.getLength() / 2; i++) {
                var hue = (m_ledHue + (i * 180 / (m_LEDBuffer.getLength() / 2))) % 180;
                m_LEDBuffer.setHSV(i, hue, 255, 128);
                m_LEDBuffer.setHSV(m_LEDBuffer.getLength() - i - 1, hue, 255, 128);
            }
            m_ledHue += 2;
            m_ledHue %= 180;

        }
        else

        {
            for (var i = 0; i < m_LEDBuffer.getLength() / 2; i++) {
                if (i == m_ledLoop) {
                    m_LEDBuffer.setRGB(i, 0, 0, 0);
                    m_LEDBuffer.setRGB(m_LEDBuffer.getLength() - i - 1, 0, 0, 0);
                }
                else {
                    m_LEDBuffer.setRGB(i, m_ledG, m_ledR, m_ledB);
                    m_LEDBuffer.setRGB(m_LEDBuffer.getLength() - i - 1, m_ledG, m_ledR, m_ledB);
                }
            }
        }

        m_LEDLight.setData(m_LEDBuffer);
        m_LEDLight.start();
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {

    }
}
