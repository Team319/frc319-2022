// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.climber;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetClimbEncoders extends InstantCommand {
  public ResetClimbEncoders() {
    addRequirements(Robot.climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.climber.resetPivotEncoderPosition();
    Robot.climber.resetTelescopeEncoderPosition();
    Robot.climber.setPivotPosition(0);
    Robot.climber.setTelescopePosition(0);
  }
}
