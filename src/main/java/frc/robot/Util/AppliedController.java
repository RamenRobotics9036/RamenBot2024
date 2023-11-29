package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;

public class AppliedController extends XboxController {
    private double controllerDeadband;

    public AppliedController(int port) {
        super(port);
        this.controllerDeadband = OperatorConstants.controllerDeadbandPercent;
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), controllerDeadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), controllerDeadband);
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), controllerDeadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), controllerDeadband);
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