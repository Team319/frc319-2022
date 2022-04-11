// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  // Runs 1 Falcon 500 & 1 Neo 550
  public CANSparkMax collectorPivotLead = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax collectorPivotFollow = new CANSparkMax(15, MotorType.kBrushless);
  public CANSparkMax collectorIntakeLead = new CANSparkMax(16, MotorType.kBrushless);
  public CANSparkMax collectorIntakeFollow = new CANSparkMax(20, MotorType.kBrushless);

  public DigitalInput homeSwitch = new DigitalInput(1);
  public DigitalInput lowerSwitch = new DigitalInput(2);

  double collectorInput = 0 ;
  public int currentLimitExtend = 20;
  public int currentLimitRetract = 25;

  public int didCurrentLimitChange = 0;
  public double filteredOutputAmps = 0;

  public LinearFilter currentFilter = LinearFilter.movingAverage(10);

  public Collector() {
    collectorIntakeLead.setInverted(false);
    collectorIntakeFollow.setInverted(true);
    // Set Motor Inversions
    //collectorPivotLead.setInverted(false);
    //collectorPivotFollow.setInverted(true);
    
    //collectorIntake.setInverted(true);

    //collectorPivotFollow.follow(collectorPivotLead);


    // collectorPivot Motor PID Setup
    //collectorPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //collectorPivot.configClosedloopRamp(0.25);
    //collectorPivot.configMotionParameters(collectorPivotParameters);

    // We default with the collector Retracted, 
    //so we want it to start expecting to extend
    collectorPivotLead.setSmartCurrentLimit(currentLimitExtend);
    collectorPivotFollow.setSmartCurrentLimit(currentLimitExtend);

    // Remote forward and reverse limit switches
    //collectorPivot.configForwardLimitSwitchSource(type, normalOpenOrClose, 0);

  }

  public double getPosition() {
    return collectorPivotLead.getEncoder().getPosition();
  }

  // Gets the current draw of the collectorPivot motor
  public double getPivotCurrentDraw() {
    return filteredOutputAmps;
  }

  // Gets the current draw of the collectorIntake motor
  public double getIntakeCurrentDraw() {
    return collectorIntakeLead.getOutputCurrent();
  }

  // Gets the volage output of the collectorPivot motor
  public double getPivotVoltageOutput() {
    return collectorPivotLead.getBusVoltage();
  }

  // Gets the volage output of the collectorIntake motor
  public double getIntakeVoltageOutput() {
    return collectorIntakeLead.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //this.collectorInput = Robot.oi.operatorController.leftStick.getX();
    //this.setCollectorPivot(ControlMode.PercentOutput, this.collectorInput * 0.5 );

    filteredOutputAmps = currentFilter.calculate(collectorPivotLead.getOutputCurrent());
  }

  // Sets the speed of the collectorIntake motor
  public void setIntake(double setpoint) {
    collectorIntakeLead.set(setpoint);
    collectorIntakeFollow.set(setpoint);
  }

  public void setCollectorPivot(double setpoint){
    collectorPivotLead.set(setpoint);
    collectorPivotFollow.set(setpoint);
  }

  public void resetEncoderPosition() {
    collectorPivotLead.getEncoder().setPosition(0);
  }
}
