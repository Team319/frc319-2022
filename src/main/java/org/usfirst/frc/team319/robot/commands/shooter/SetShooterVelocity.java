// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetShooterVelocity extends InstantCommand {
  double velocity;
  /** Creates a new SetShooterVelocity. */
  public SetShooterVelocity(double setpoint) {
    addRequirements(Robot.shooter);

    this.velocity = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter Target", this.velocity);
    //Robot.shooter.setShooterVelocity(velocity);
    Robot.shooter.shoot();
  }

}
