package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.AppliedController;

public class LEDSystem extends SubsystemBase {

    // I WANT TO USE THE TOP INTAKE MOTOR BECAUSE THAT WILL ONLY CHANGE IF IT ACTUALLY GRABS A PIECE

    // THE BOTTOM NOTE MIGHT SLOW DOWN BECAUSE IT HITS THE NOTE, BUT IT MAY NOT ACTUALLY INTAKE IT
    // private final CANSparkMax intakeMotor = new CANSparkM
    // IntakeConstants.intakeMotorRight
    // MotorType.kBrushless);
    // private RelativeEncoder m_encoder = intakeMotor.getEncoder();

    private Timer m_timer = new Timer();

    private DigitalInput beamBreak = new DigitalInput(1);

    private AddressableLED m_LEDLight = new AddressableLED(
            Constants.OperatorConstants.kLEDLightsChannel);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(
            Constants.OperatorConstants.kLEDLightsLength);

    private AppliedController m_driveController;
    private AppliedController m_armController;

    private int m_ledLoop;
    private int m_ledR;
    private int m_ledG;
    private int m_ledB;
    private int m_ledHue;

    private boolean noteInIntake;

    public LEDSystem(AppliedController armController, AppliedController driveController) {

        m_driveController = driveController;
        m_armController = armController;

        // initShuffleBoard();

        m_ledLoop = 0;
        m_ledR = 255;
        m_ledG = 0;
        m_ledB = 0;
        m_ledHue = 0;
        m_LEDLight.setLength(m_LEDBuffer.getLength());

    }

    public void resetLED() {
        m_ledR = 255;
        m_ledG = 0;
        m_ledB = 0;
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Intake")
                .add("Beam Break: ", beamBreak.get());

    }

    @Override
    public void periodic() {

        if (beamBreak.get()) {
            for (int i = 0; i < OperatorConstants.kLEDLightsLength; i++) {
                m_LEDBuffer.setRGB(i, 255, 0, 0);
            }
            m_armController.setRumble(RumbleType.kBothRumble, .7);
            if (m_timer.get() < 2 && noteInIntake) {
                m_timer.restart();
            }

        }
        else {
            for (int i = 0; i < OperatorConstants.kLEDLightsLength; i++) {
                m_LEDBuffer.setRGB(i, 0, 255, 0);
            }
            m_armController.setRumble(RumbleType.kBothRumble, .7);
        }

        if (m_timer.get() >= 2) {
            m_armController.setRumble(RumbleType.kBothRumble, 0);
        }

        m_LEDLight.setData(m_LEDBuffer);
        m_LEDLight.start();
    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        m_LEDBuffer.setRGB(0, 0, 0, 0);
        m_LEDLight.setData(m_LEDBuffer);
        m_LEDLight.start();
        m_armController.setRumble(RumbleType.kBothRumble, 0);
        m_driveController.setRumble(RumbleType.kBothRumble, 0);

    }
}
