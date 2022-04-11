// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.cargoTunnel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc.team319.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TunnelPO extends InstantCommand {
  // Cargo Tunnel Percent Output

  private double tunnelSetpoint;

  public TunnelPO(double setpoint) {
    addRequirements(Robot.cargoTunnel);
    this.tunnelSetpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.cargoTunnel.set(tunnelSetpoint, tunnelSetpoint);
  }
}
