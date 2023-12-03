package frc.robot.Util;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AppliedEncoder extends AnalogEncoder {
    private double m_lastPosition;
    private double m_lastUpdate;

    public AppliedEncoder(int channel) {
        super(channel);
        updateLastPosition();
        updateTime();
    }

    public void updateTime() {
        m_lastUpdate = System.currentTimeMillis() / 1000.0;
    }

    public void updateLastPosition() {
        m_lastPosition = super.getAbsolutePosition();
    }

    public double getRate() {
        double rate = super.getAbsolutePosition() - m_lastPosition;
        rate = rate / (m_lastUpdate);
        updateLastPosition();
        updateTime();
        return rate;
    }
}
