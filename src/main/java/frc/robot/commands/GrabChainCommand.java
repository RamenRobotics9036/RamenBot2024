package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GrabChainConstants;
import frc.robot.subsystems.HookSystem;
import frc.robot.subsystems.SwerveDriveSystem;

public class GrabChainCommand extends Command {

    private SwerveDriveSystem m_swerve;
    private HookSystem m_hookSystem;
    private Timer m_timer = new Timer();
    private int m_phase = 1;
    private double m_oldPos;

    public GrabChainCommand(HookSystem hook, SwerveDriveSystem swerve) {
        m_hookSystem = hook;
        m_swerve = swerve;
        addRequirements(m_hookSystem, m_swerve);

    }

    @Override
    public void initialize() {

        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_phase == 1) {
            m_hookSystem.setHookSpeed(GrabChainConstants.hookSpeed);
            if (m_hookSystem.getLeadEncoderValue() >= GrabChainConstants.hookRotationsNeeded) {
                m_hookSystem.setHookSpeed(0);
                m_phase += 1;
                m_oldPos = m_swerve.getFrontLeftDriveEncoder();
            }
        }
        else if (m_phase == 2) {
            m_swerve.drive(0, GrabChainConstants.swerveSpeed, 0, false);
            if (m_swerve.getFrontLeftDriveEncoder()
                    - m_oldPos >= GrabChainConstants.swerveRotationsNeeded) {
                m_swerve.drive(0, 0, 0);
                m_phase += 1;
            }
        }
        else if (m_phase == 3) {
            m_hookSystem.setHookSpeed(-GrabChainConstants.hookSpeed);
            if (m_hookSystem.getLeadEncoderValue() >= GrabChainConstants.hookRotationsNeededFinal) {
                m_hookSystem.setHookSpeed(0);
                m_phase += 1;
            }
        }

    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= GrabChainConstants.maxTime || m_phase == 4) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hookSystem.stopSystem();
    }

}
