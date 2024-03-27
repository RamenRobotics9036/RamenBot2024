package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.util.AppliedController;

public class ChargeShootCommand extends Command {
    private ShooterSystem m_shooterSystem;
    private AppliedController m_controller;
    private EventLoop eventLoop = new EventLoop();

    public ChargeShootCommand(ShooterSystem shooter, AppliedController controller) {
        m_shooterSystem = shooter;
        m_controller = controller;
        addRequirements(m_shooterSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_controller.povLeft(eventLoop).getAsBoolean()) {
            m_shooterSystem.setShootSpeed(ShooterConstants.shooterSpeed);
        }
        else {
            m_shooterSystem.setShootSpeed(ShooterConstants.lowShooterSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSystem.stopSystem();
    }
}
