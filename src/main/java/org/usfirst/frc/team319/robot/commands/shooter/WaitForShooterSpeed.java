// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WaitForShooterSpeed extends InstantCommand {
  double velocity;
  double threshhold;
  /** Creates a new SetShooterVelocity. */
  public WaitForShooterSpeed(double setpoint, double threshhold) {
    addRequirements(Robot.shooter);

    velocity = setpoint;
    this.threshhold = threshhold;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.waitForShooterSpeed(velocity, threshhold);
  }

}
