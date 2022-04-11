/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot.commands.drivetrain;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDoNothing extends CommandBase {
  public DrivetrainDoNothing() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

}
