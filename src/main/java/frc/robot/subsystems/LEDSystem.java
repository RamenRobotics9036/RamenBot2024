package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("AbbreviationAsWordInNameCheck")
public class LEDSystem extends SubsystemBase {

    // I WANT TO USE THE TOP INTAKE MOTOR BECAUSE THAT WILL ONLY CHANGE IF IT ACTUALLY GRABS A PIECE

    // THE BOTTOM NOTE MIGHT SLOW DOWN BECAUSE IT HITS THE NOTE, BUT IT MAY NOT ACTUALLY INTAKE IT
    // private final CANSparkMax intakeMotor = new CANSparkM
    // IntakeConstants.intakeMotorRight
    // MotorType.kBrushless);
    // private RelativeEncoder m_encoder = intakeMotor.getEncoder();

    // ARBITRARY NUMBER
    private double m_normalCurrent = Double.MAX_VALUE;

    private AddressableLED m_ledLight = new AddressableLED(
            Constants.OperatorConstants.kLEDLightsChannel);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(
            Constants.OperatorConstants.kLEDLightsLength);

    private IntakeSystem m_intakeSystem;

    public LEDSystem(IntakeSystem intakeSystem) {

        // intakeMotor.restoreFactoryDefaults();
        // intakeMotor.setSmartCurrentLimit(IntakeConstants.smartCurrentLimit);
        // initShuffleBoard();

        m_ledLight.setLength(m_ledBuffer.getLength());

        m_intakeSystem = intakeSystem;
    }

    /**
     * Sets the LED lights to yellow.
     */
    public void setLedsYellow() {
    }

    public void resetLED() {
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
        if (m_intakeSystem.getOutputCurrent() > m_normalCurrent) {
            // m_LEDBuffer.setHSV()
            // WILL ONLY CHANGE ONE LIGHT FOR NOW. ADD A FOR LOOP
            // TO ACTUALLY CHANGE ALL OF THE LIGHTS LATER
            m_ledBuffer.setRGB(0, 0, 255, 0);

        }

        // THINK ABOUT HOW IT WILL KNOW IF IT DOESNT HAVE A PIECE
        // for now, when it shoots, thats when it will know that it used a piece. I can check this
        // by other checking the velocity of the shoot motors or by seeing if the shoot command was
        // run

        // If shootsystem.usedcommand() == True {m_LEDBuffer.setRGB(0, 255, 0, 0)};

        // I WILL ALSO HAVE TO THINK ABOUT THE INTERACTION WITH THE AMP LIGHT

        m_ledLight.setData(m_ledBuffer);
        m_ledLight.start();
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        m_ledBuffer.setRGB(0, 0, 0, 0);
        m_ledLight.setData(m_ledBuffer);
        m_ledLight.start();
    }
}
