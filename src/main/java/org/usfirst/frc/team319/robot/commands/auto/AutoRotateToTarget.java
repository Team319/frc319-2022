// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team319.models.DriveMode;
import org.usfirst.frc.team319.models.DriveSignal;
import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.utils.BobDriveHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoRotateToTarget extends CommandBase {
  /** Creates a new AutoRotateToTarget. */

  BobDriveHelper helper;
	private double quickTurnThreshold = 0.2;

  double rotateValue = 0;
  double moveValue = 0;
  boolean quickTurn = false;

  private PIDController limelightRotatePID = new PIDController(0.25, 0.0, 0.01);

  double threshold = 0;

  public AutoRotateToTarget(double threshold_) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.limelight);
    addRequirements(Robot.drivetrain);

    this.threshold = threshold_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.setMode(DriveMode.Limelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotateValue = -limelightRotatePID.calculate(Robot.limelight.getXProportional()) ;
    
    moveValue = 0;
    quickTurn = (moveValue < quickTurnThreshold && moveValue > -quickTurnThreshold);	
    DriveSignal driveSignal = helper.cheesyDrive(moveValue, rotateValue, quickTurn, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    DriveSignal driveSignal = helper.cheesyDrive(0, 0, quickTurn, false);
    Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
    System.out.println("Limelight Rotation Value" + Robot.limelight.getXProportional());
    Robot.drivetrain.setMode(DriveMode.Normal);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Robot.limelight.getXProportional() < this.threshold);  // this is going to be scuffed... but fingers crossed...
  }
}
