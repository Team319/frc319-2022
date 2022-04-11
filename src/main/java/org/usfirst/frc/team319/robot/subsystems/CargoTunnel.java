// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoTunnel extends SubsystemBase {
  // Runs 2 Neo 550's
  public CANSparkMax cargoLead = new CANSparkMax(17, MotorType.kBrushless);
  public CANSparkMax cargoFollow = new CANSparkMax(18, MotorType.kBrushless);
  public CargoTunnel() {
    cargoLead.restoreFactoryDefaults();
    cargoFollow.restoreFactoryDefaults();

    cargoLead.setInverted(false);
    cargoFollow.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets the speed of the motors
  public void set(double leftSetpoint, double rightSetPoint) {
    setLead(leftSetpoint);
    setFollow(rightSetPoint);
  }

  // Sets the speed of the lead motor
  public void setLead(double setpoint) {
    this.cargoLead.set(setpoint);
  }

  // Sets the speed of the follow motor
  public void setFollow(double setpoint) {
    cargoFollow.set(setpoint);
  }

  public double getLeadCurrentDraw() {
    return this.cargoLead.getOutputCurrent();
  }

  public double getFollowCurrentDraw() {
    return this.cargoFollow.getOutputCurrent();
  }
}