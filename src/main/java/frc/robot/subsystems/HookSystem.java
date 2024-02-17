package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.Constants.HookConstants;

public class HookSystem extends SubsystemBase {
    private CANSparkMax m_leaderMotor = new CANSparkMax(HookConstants.leftHookCANId,
            MotorType.kBrushless);
    private CANSparkMax m_followerMotor = new CANSparkMax(HookConstants.rightHookCANId,
            MotorType.kBrushless);
    private RelativeEncoder m_lEncoder = m_leaderMotor.getEncoder();
    private RelativeEncoder m_rEncoder = m_followerMotor.getEncoder();

    public HookSystem() {
        m_followerMotor.follow(m_leaderMotor);
    }

    public double getLeadEncoderValue() {
        return m_lEncoder.getPosition();
    }

    public double getFollowEncoderValue() {
        return m_rEncoder.getPosition();
    }

    public void setHookSpeed(double speed) {
        speed = MathUtil
                .clamp(speed, -HookConstants.maxOutputPercent, HookConstants.maxOutputPercent);
        if ((speed < 0 && getLeadEncoderValue() < SetArmConstants.armMin)
                || (speed > 0 && getLeadEncoderValue() > SetArmConstants.armMax)) {
            m_leaderMotor.set(speed);
        }
        else {
            m_leaderMotor.set(0);
        }
    }

    public void setHookSpeedAdmin(double speed) {
        speed = MathUtil
                .clamp(speed, -HookConstants.maxOutputPercent, HookConstants.maxOutputPercent);
        m_leaderMotor.set(speed);
    }

    public double getLeadMotorSpeed() {
        return m_leaderMotor.get();
    }

    public double getFollowMotorSpeed() {
        return m_followerMotor.get();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Hook").addDouble("Leader Hook Value: ", () -> getLeadEncoderValue());
        Shuffleboard.getTab("Hook")
                .addDouble("Follower Hook Value: ", () -> getFollowEncoderValue());
        Shuffleboard.getTab("Hook").addDouble("Leader Hook Speed: ", () -> getLeadMotorSpeed());
        Shuffleboard.getTab("Hook").addDouble("Follower Hook Speed: ", () -> getFollowMotorSpeed());

    }

    public void stopSystem() {
        m_leaderMotor.stopMotor();

    }

}
