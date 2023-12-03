// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The `Robot` class extends `TimedRobot` and is the main class for controlling the robot's
 * operations. It initializes the robot, runs periodic tasks, and handles the teleoperation
 * control period of the robot. It uses a `RobotContainer` instance to manage the robot's
 * subsystems and commands.
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
    m_robotContainer.bindCommands();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }
}
