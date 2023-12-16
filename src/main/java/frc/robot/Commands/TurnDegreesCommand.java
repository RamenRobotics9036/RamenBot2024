package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesCommand extends CommandBase {
  private Timer m_timer;
  
  public TurnDegreesCommand() {
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public void execute() {
      System.out.println("Timer: " + m_timer.get());
  }

  @Override
  public boolean isFinished() {
      if (m_timer.get() > 3) {
        return true;
      }
      return false;
  }
  
  @Override
  public void end(boolean interrupted) {
      m_timer.stop();
  }
}

