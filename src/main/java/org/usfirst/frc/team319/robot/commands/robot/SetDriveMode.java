
package org.usfirst.frc.team319.robot.commands.robot;

import org.usfirst.frc.team319.models.DriveMode;
import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetDriveMode extends CommandBase {
  private DriveMode driveMode = DriveMode.Normal;

  public SetDriveMode(DriveMode driveMode) {
    this.driveMode = driveMode;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.drivetrain.mode = this.driveMode;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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
