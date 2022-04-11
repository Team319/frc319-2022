// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.subsystems;

import org.usfirst.frc.team319.models.BobTalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc.team319.models.MotionParameters;
import org.usfirst.frc.team319.models.PIDGains;
import org.usfirst.frc.team319.models.RobotMode;
import org.usfirst.frc.team319.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climber extends SubsystemBase {
  public int PIVOT_SLOT = 0;
  public int TELESCOPE_SLOT = 1;
  // Runs 4 Falcon 500's

  // Left Motors
  public BobTalonFX leftPivot = new BobTalonFX(10);
  public BobTalonFX leftTelescope = new BobTalonFX(11);

  // Right Motors
  public BobTalonFX rightPivot = new BobTalonFX(12);
  public BobTalonFX rightTelescope = new BobTalonFX(13);

  private PIDGains pivotGains = new PIDGains(PIVOT_SLOT, 0.01, 0, 0, 0, 0);
  private MotionParameters pivotMotionParameters = new MotionParameters(0, 0, pivotGains);

  private PIDGains telescopeGains = new PIDGains(TELESCOPE_SLOT, 0, 0, 0, 0, 0);
  private MotionParameters telescopeMotionParameters = new MotionParameters(0, 0, telescopeGains);
  
  public double telescopeInput;
  public double pivotInput;

  public final int negPivotLimit = 0;
  public final int posPivotLimit = 70000;
  public final int posTelescopeLimit = 242000;
  public final int telescopeZero = -200;
  public final double telescopeSpeedMod = 1.0;
  public final double pivotSpeedMod = 0.25;

  public Climber() {
    // Set Left Motor Inversion
    this.leftPivot.setInverted(true);
    this.leftTelescope.setInverted(true);

    // Set Right Motor Inversion
    this.rightPivot.setInverted(false);
    this.rightTelescope.setInverted(false);

    // leftPivot Motor PID Setup
    this.leftPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.leftPivot.configClosedloopRamp(0.25);
    this.leftPivot.configMotionParameters(pivotMotionParameters);

    // leftTelecope Motor PID Setup
    this.leftTelescope.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.leftTelescope.configClosedloopRamp(0.25);
    this.leftTelescope.configMotionParameters(telescopeMotionParameters);

    // rightPivot Motor PID Setup
    this.rightPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.rightPivot.configClosedloopRamp(0.25);
    this.rightPivot.configMotionParameters(pivotMotionParameters);

    // rightTelecope Motor PID Setup
    this.rightTelescope.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.rightTelescope.configClosedloopRamp(0.25);
    this.rightTelescope.configMotionParameters(telescopeMotionParameters);

    leftTelescope.setNeutralMode(NeutralMode.Brake);
    rightTelescope.setNeutralMode(NeutralMode.Brake);
    rightPivot.setNeutralMode(NeutralMode.Brake);
    leftPivot.setNeutralMode(NeutralMode.Brake);
  }

  public double getTelescopeCurrentDraw() {
    return rightTelescope.getSupplyCurrent();
  }

  public double getTelescopePosition() {
    return rightTelescope.getSelectedSensorPosition();
  }

  public double getRightPivotPosition() {
    return rightPivot.getSelectedSensorPosition();
  }

  public double getLeftPivotPosition() {
    return leftPivot.getSelectedSensorPosition();
  }

  public void setTelescope(ControlMode controlMode, double setpoint) {
    rightTelescope.set(controlMode, setpoint);
  }
  
  public void setPivot(ControlMode controlMode, double setpoint) {
    rightPivot.set(controlMode, setpoint);
  }

  public void setPivotPosition(double setpoint) {
    rightPivot.set(ControlMode.Position, setpoint);
  }

  public void pivotPercentOutput(double setpoint) {
    setPivot(ControlMode.PercentOutput, setpoint);
  }

  public void setTelescopePosition(double setpoint) {
    setTelescope(ControlMode.Position, setpoint);
  }

  public void resetTelescopeEncoderPosition() {
    rightTelescope.setSelectedSensorPosition(0);
    leftTelescope.setSelectedSensorPosition(0);
  }

  public void resetPivotEncoderPosition() {
    rightPivot.setSelectedSensorPosition(0);
    leftPivot.setSelectedSensorPosition(0);
  }

  public void moveClimberManually(){
    //telescopeInput = -Robot.oi.operatorController.rightStick.getY();
    //pivotInput = Robot.oi.operatorController.leftStick.getY();
    this.setTelescope(ControlMode.PercentOutput, telescopeInput * telescopeSpeedMod);
    this.setPivot(ControlMode.PercentOutput, pivotInput * pivotSpeedMod);
  }

  @Override
  public void periodic() {


    if (Robot.robotMode == RobotMode.Climb) {
      this.telescopeInput = -Robot.oi.operatorController.rightStick.getY();
      this.pivotInput = Robot.oi.operatorController.leftStick.getY();

      /**** Code for the pivot soft limits and manual control ****/

      // allow to move within the limits
      if(this.getRightPivotPosition() < posPivotLimit && this.getRightPivotPosition() > negPivotLimit)
      {
        this.setPivot(ControlMode.PercentOutput, pivotInput * pivotSpeedMod);
      }
      // if above positive limit but moving back then allow it to move
      else if(this.getRightPivotPosition() > posPivotLimit && this.pivotInput < 0)
      {
        this.setPivot(ControlMode.PercentOutput, pivotInput * pivotSpeedMod);
      }
      // if below negative limit but moving forward then allow it to move
      else if(this.getRightPivotPosition() < negPivotLimit && this.pivotInput > 0)
      {
        this.setPivot(ControlMode.PercentOutput, pivotInput * pivotSpeedMod);
      }
      // if fail all conditions then do not allow movement
      else
      {
        this.setPivot(ControlMode.PercentOutput, 0);
      }

      /**** Code for the telescope soft limits and manual control ****/

      if(this.getTelescopePosition() < posTelescopeLimit && this.getTelescopePosition() >= telescopeZero){
        this.setTelescope(ControlMode.PercentOutput, telescopeInput * telescopeSpeedMod);
      }
      else if(this.getTelescopePosition() > posTelescopeLimit && this.telescopeInput < 0){
        this.setTelescope(ControlMode.PercentOutput, telescopeInput * telescopeSpeedMod);
      }
      else if(this.getTelescopePosition() < telescopeZero && this.telescopeInput > 0){
        this.setTelescope(ControlMode.PercentOutput, telescopeInput * telescopeSpeedMod);
      }
      else{
        this.setTelescope(ControlMode.PercentOutput, 0);
      }
    }
    // This method will be called once per scheduler run
  }
}
