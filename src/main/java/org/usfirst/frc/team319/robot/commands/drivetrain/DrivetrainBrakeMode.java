// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.drivetrain;

import org.usfirst.frc.team319.robot.Robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetrainBrakeMode extends InstantCommand {

  NeutralMode driveMode;

  public DrivetrainBrakeMode( NeutralMode driveMode_) {
    // Use addRequirements() here to declare subsystem dependencies.
		addRequirements(Robot.drivetrain);

    this.driveMode = driveMode_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
		Robot.drivetrain.setNeutralMode(this.driveMode);
	}


  
}
