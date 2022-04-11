// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;



public class DrivetrainDriveDistance extends CommandBase {
  /** Creates a new DrivetrainDriveDistance. */

  private double distance;
  private double percentOutput;

  public DrivetrainDriveDistance(double distance_, double percentOutput_) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drivetrain);
    this.distance = distance_;
    this.percentOutput = percentOutput_; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot.drivetrain.drive(ControlMode.PercentOutput, percentOutput, percentOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drivetrain Drive Cmd");
    Robot.drivetrain.drive(ControlMode.PercentOutput, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((distance > 0) && (percentOutput > 0)) 
    {
      
      return ( ( Robot.drivetrain.leftLead.getSelectedSensorPosition() >= distance) 
            && (Robot.drivetrain.rightLead.getSelectedSensorPosition() >= distance) ) ;
    }
    if ((distance < 0) && (percentOutput < 0))
    {
      return ( (Robot.drivetrain.leftLead.getSelectedSensorPosition() <= distance) 
            && (Robot.drivetrain.rightLead.getSelectedSensorPosition() <= distance)  ) ;
    }
    else
    {
      System.out.println("You used this function wrong and you should feel bad");
      return true;
    }
    
  }
}
