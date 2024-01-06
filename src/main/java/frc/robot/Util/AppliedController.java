package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;

public class AppliedController extends XboxController {
    private double controllerDeadband;
    private double controllerExponent;

    public AppliedController(int port) {
        super(port);
        this.controllerDeadband = OperatorConstants.controllerDeadbandPercent;
        this.controllerExponent = Math.pow(OperatorConstants.controllerExpo, OperatorConstants.controllerExpoRatio);
    }

    public double expo(double input){
        double expo = Math.pow(Math.abs(input), controllerExponent);
        return input < 0 ? -expo : expo;
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(expo(super.getLeftY()), controllerDeadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(expo(super.getRightY()), controllerDeadband);
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(expo(super.getLeftX()), controllerDeadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(expo(super.getRightX()), controllerDeadband);
    }

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