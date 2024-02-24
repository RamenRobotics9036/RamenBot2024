// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot is timed, meaning that it will run the periodic methods at a fixed of 20ms.
 */
public class Robot extends TimedRobot {
    private RobotContainer m_robotContainer = new RobotContainer();

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.resetGyroTeleop();
        m_robotContainer.bindCommands();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.scheduleAutonomousCommand();

        // initializes auto based off shuffleboard

        // may have to specify autobuilder properties via code
        // m_robotContainer.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }
}
