// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team319.robot.Robot;

public class WaitForShooterVelocity extends CommandBase {
  private Timer timer = new Timer();
  private double minWaitSeconds = 0.0;
  private double minVelocity = 0.0;
  private double maxWaitSeconds = 0.0;

  public WaitForShooterVelocity(double minVelocity, double minWaitSeconds, double maxWaitSeconds) {
    addRequirements(Robot.shooter);
    this.minWaitSeconds = minWaitSeconds;
    this.minVelocity = minVelocity;
    this.maxWaitSeconds = maxWaitSeconds;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double shooterVelocity = Robot.shooter.getShooterVelocity();
    boolean hasAchievedVelocity = shooterVelocity >= minVelocity;
    boolean hasTimeElapsed = timer.get() >= minWaitSeconds;
    boolean cantWaitAnymore = timer.get() >= maxWaitSeconds;
    return (hasTimeElapsed && hasAchievedVelocity) || cantWaitAnymore;
  }
}
