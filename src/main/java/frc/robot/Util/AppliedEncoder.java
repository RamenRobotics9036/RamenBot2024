package frc.robot.Util;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AppliedEncoder extends AnalogEncoder {
    private double m_lastPosition;

    public AppliedEncoder(int channel) {
        super(channel);
        m_lastPosition = super.getAbsolutePosition();
    }

    public double getRate() {
        double rate = super.getAbsolutePosition() - m_lastPosition;
        m_lastPosition = super.getAbsolutePosition();
        return rate;
    }
}
