/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot.commands.limelight;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnLedOn extends CommandBase {
  public TurnLedOn() {
    // Use requires() here to declare subsystem dependencies
     addRequirements(Robot.limelight);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.limelight.setLedModeOn();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
