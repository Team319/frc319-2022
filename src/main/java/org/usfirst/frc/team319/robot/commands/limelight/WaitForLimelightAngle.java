/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot.commands.limelight;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.shooter.SmartShoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForLimelightAngle extends CommandBase {

  public double angle = 0;
  public int goodSignalCounts = 0;
  public int badSignalCounts = 0;
  public int debounceThreshold = 10;

  public WaitForLimelightAngle(double degrees) {
    this.angle = degrees;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // System.out.println("angle in feet " + Robot.limelight.getangle());
    // Robot.limelight.setLedModeOn();
    double limelightX = Robot.limelight.getAngleDegrees();
    SmartDashboard.putNumber("limeLightAngle", limelightX);
    if ( (limelightX <= angle) && (limelightX >= -angle) )
    {
      goodSignalCounts++;
      badSignalCounts = 0;
    }
    else
    {
      badSignalCounts++;
      goodSignalCounts = 0;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    
    return goodSignalCounts >= debounceThreshold;
  }

 

  
}
