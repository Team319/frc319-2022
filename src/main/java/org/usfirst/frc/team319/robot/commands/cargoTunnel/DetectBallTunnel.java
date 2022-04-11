// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.cargoTunnel;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DetectBallTunnel extends CommandBase {
  double threshold;
  /** Creates a new DetectBallTunnel. */
  public DetectBallTunnel(double threshold) {
    addRequirements(Robot.cargoTunnel);
    this.threshold = threshold;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    new WaitForTime(0.2);
    if((Robot.cargoTunnel.getLeadCurrentDraw() >= threshold) && (Robot.cargoTunnel.getFollowCurrentDraw() >= threshold))
    {
      return true; 
    }
    else
    {
      return false;
    }
  }
}
