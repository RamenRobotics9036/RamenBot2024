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

    private Timer m_timer = new Timer();
    private boolean m_isRumbling;

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
        m_isRumbling = false;
        m_driveController = driveController;
        m_armController = armController;

        // initShuffleBoard();

        m_ledLoop = 0;
        m_ledR = 255;
        m_ledG = 0;
        m_ledB = 0;
        m_ledHue = 0;
        m_LEDLight.setLength(m_LEDBuffer.getLength());
        // m_LEDLight.start();

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
        // if (m_isRumbling) {
        // if (m_timer.get() >= 2) {
        // m_armController.setRumble(RumbleType.kBothRumble, 0);
        // m_isRumbling = false;
        // }
        // }
        // else if (!beamBreak.get()) {
        // m_timer.restart();
        // m_armController.setRumble(RumbleType.kBothRumble, 0.3);
        // m_isRumbling = true;
        // }

        // sees the light (does not have note)
        if (beamBreak.get()) {
            for (int i = 0; i < OperatorConstants.kLEDLightsLength; i++) {
                m_LEDBuffer.setRGB(i, 255, 0, 0);
            }
            // m_armController.setRumble(RumbleType.kBothRumble, 0);
        }
        // // does not see the light (has the note)
        else {
            for (int i = 0; i < OperatorConstants.kLEDLightsLength; i++) {
                m_LEDBuffer.setRGB(i, 0, 255, 0);
            }
            // m_armController.setRumble(RumbleType.kBothRumble, .3);
        }

        // if (m_timer.get() >= 2) {
        // m_armController.setRumble(RumbleType.kBothRumble, 0);
        // }

        m_LEDLight.setData(m_LEDBuffer);
        m_LEDLight.start();

    }

    /**
     * Stop the intake system.
     */
    public void stopSystem() {
        // m_LEDBuffer.setRGB(0, 0, 0, 0);
        // m_LEDLight.setData(m_LEDBuffer);
        // m_LEDLight.start();
        m_armController.setRumble(RumbleType.kBothRumble, 0);
        m_driveController.setRumble(RumbleType.kBothRumble, 0);

    }
}
