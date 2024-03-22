package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;
import frc.robot.commands.DefaultHookCommand;
import frc.robot.util.AppliedController;

public class HookSystem extends SubsystemBase {
    private CANSparkMax rightHookMotor = new CANSparkMax(HookConstants.rightHookCANId,
            MotorType.kBrushless);
    private CANSparkMax leftHookMotor = new CANSparkMax(HookConstants.leftHookCANId,
            MotorType.kBrushless);
    private RelativeEncoder m_lEncoder = leftHookMotor.getEncoder();
    private RelativeEncoder m_rEncoder = rightHookMotor.getEncoder();
    private AppliedController m_controller;

    public HookSystem(AppliedController controller) {
        m_controller = controller;
        leftHookMotor.restoreFactoryDefaults();
        rightHookMotor.restoreFactoryDefaults();
        leftHookMotor.setIdleMode(IdleMode.kBrake);
        rightHookMotor.setIdleMode(IdleMode.kBrake);

        leftHookMotor.setSmartCurrentLimit(15);
        rightHookMotor.setSmartCurrentLimit(15);

        leftHookMotor.setInverted(false);
        rightHookMotor.setInverted(false);

        // rightHookMotor.follow(leftHookMotor);

        // initShuffleBoard();
        setDefaultCommand(new DefaultHookCommand(this, m_controller));
    }

    public double getLeadEncoderValue() {
        return m_lEncoder.getPosition();
    }

    public double getFollowEncoderValue() {
        return m_rEncoder.getPosition();
    }

    // DOES NOT GET USED
    // public void setHookSpeed(double speed) {
    // speed = MathUtil
    // .clamp(speed, -HookConstants.maxOutputPercent, HookConstants.maxOutputPercent);
    // leftHookMotor.set(speed);
    // }

    public void setHookSpeedRight(double speed) {
        speed = MathUtil
                .clamp(speed, -HookConstants.maxOutputPercent, HookConstants.maxOutputPercent);
        leftHookMotor.set(speed);
    }

    public void setHookSpeedLeft(double speed) {
        speed = MathUtil
                .clamp(speed, -HookConstants.maxOutputPercent, HookConstants.maxOutputPercent);
        rightHookMotor.set(speed);
    }

    public void setHookSpeedAdmin(double speed) {
        leftHookMotor.set(speed);
    }

    public double getLeadMotorSpeed() {
        return leftHookMotor.get();
    }

    public double getFollowMotorSpeed() {
        return rightHookMotor.get();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Hook").addDouble("Leader Hook Value: ", () -> getLeadEncoderValue());
        Shuffleboard.getTab("Hook")
                .addDouble("Follower Hook Value: ", () -> getFollowEncoderValue());
        Shuffleboard.getTab("Hook").addDouble("Leader Hook Speed: ", () -> getLeadMotorSpeed());
        Shuffleboard.getTab("Hook").addDouble("Follower Hook Speed: ", () -> getFollowMotorSpeed());

    }

    public void stopSystem() {
        leftHookMotor.stopMotor();
        rightHookMotor.stopMotor();

    }

}
