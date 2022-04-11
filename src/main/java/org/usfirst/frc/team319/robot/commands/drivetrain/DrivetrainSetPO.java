// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainSetPO extends InstantCommand {
  /** Creates a new DrivetrainSetPO. */
  double setpoint;
  public DrivetrainSetPO(double setpoint) {
    addRequirements(Robot.drivetrain);
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.drive(ControlMode.PercentOutput, -setpoint, -setpoint);
  }
}
