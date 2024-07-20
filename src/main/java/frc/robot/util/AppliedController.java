package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;

/**
 * AppliedController.
 */
public class AppliedController extends XboxController {
    private double m_controllerDeadband;
    private double m_controllerExponent;

    /**
     * Constructor.
     */
    public AppliedController(int port) {
        super(port);
        this.m_controllerDeadband = OperatorConstants.controllerDeadbandPercent;
        this.m_controllerExponent = Math.pow(
                OperatorConstants.controllerExpo,
                OperatorConstants.controllerExpoRatio);
    }

    public double expo(double input) {
        double expo = Math.pow(Math.abs(input), m_controllerExponent);
        return input < 0 ? -expo : expo;
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(expo(super.getLeftY()), m_controllerDeadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(expo(super.getRightY()), m_controllerDeadband);
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(expo(super.getLeftX()), m_controllerDeadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(expo(super.getRightX()), m_controllerDeadband);
    }

    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(expo(super.getLeftTriggerAxis()), m_controllerDeadband);
    }

    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(expo(super.getRightTriggerAxis()), m_controllerDeadband);
    }

    /**
     * Returns true if any of the joystick axes are not zero.
     */
    public boolean commandCancel() {
        if (getLeftY() != 0) {
            return true;
        }
        if (getRightY() != 0) {
            return true;
        }
        if (getLeftX() != 0) {
            return true;
        }
        if (getRightX() != 0) {
            return true;
        }
        return false;
    }
}
