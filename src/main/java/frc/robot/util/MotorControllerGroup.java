package frc.robot.util;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;

public class MotorControllerGroup {

    public ArrayList<CANSparkMax> m_motors;
    public double m_speed;

    public MotorControllerGroup(CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            m_motors.add(motor);
        }
    }

    public void set(double speed) {
        for (CANSparkMax motor : m_motors) {
            motor.set(speed);
        }
    }

    public double get() {
        return m_speed;
    }
}
