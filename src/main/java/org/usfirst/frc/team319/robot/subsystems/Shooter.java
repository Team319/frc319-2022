// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.subsystems;

import org.usfirst.frc.team319.models.BobTalonFX;
import org.usfirst.frc.team319.models.DriveMode;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.usfirst.frc.team319.models.MotionParameters;
import org.usfirst.frc.team319.models.PIDGains;
import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  int shooterPIDSlot = 0;
  private double shooterSpeed = 7000;  //Close shot = 7000, Far shot = 8500
  public int tdShootSpeed = 6000;
  public int tdFeedSpeed = 1000;
  public int shooterEjectSpeed = 500;
  public int tdEjectSpeed = 1500; 
  public double antiBackspinPercent = 1;  //value from -1.0 to 1.0, were at 0.8 for 3 feet

  /** Creates a new Shooter. */
  public BobTalonFX shooterLead = new BobTalonFX(7);
  public BobTalonFX shooterFollow = new BobTalonFX(8);
  public BobTalonFX shooterTD = new BobTalonFX(9); // This motor passes cargo to the shooter
  public CANSparkMax shooterAntiBackspin = new CANSparkMax(19, MotorType.kBrushless);

  public DigitalInput beamBreak = new DigitalInput(0);

  public boolean velocityReached = getShooterVelocity() >= shooterSpeed;

  // Tune Shooter PID p is 0.2 i is 0.00001
  private final PIDGains shooterGains = new PIDGains(shooterPIDSlot, 0.1, 0.001, 5, 1023/20660.0, 300); // should have izone
  private MotionParameters shooterMotionParameters = new MotionParameters(0, 0, shooterGains);

  public Shooter() {
    // Setting shooter motors to inverted
    shooterLead.setInverted(true);
    shooterFollow.setInverted(false);
    shooterTD.setInverted(false);
    shooterAntiBackspin.setInverted(false);

    shooterLead.setNeutralMode(NeutralMode.Coast);
    shooterFollow.setNeutralMode(NeutralMode.Coast);
    shooterTD.setNeutralMode(NeutralMode.Coast);

    // shooterLead motor PID setup
    this.shooterLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.shooterLead.configClosedloopRamp(0.25);
    this.shooterLead.configMotionParameters(shooterMotionParameters);

    this.shooterTD.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.shooterTD.configClosedloopRamp(0.25);
    this.shooterTD.configMotionParameters(shooterMotionParameters);

    this.shooterLead.configMotionSCurveStrength(0);

  }

  public double getShooterSpeed(){
    return this.shooterSpeed;
  }

  public void setShooterSpeed(double setpoint){
    this.shooterSpeed = setpoint;
  }

  public double getShooterVelocity() {
    return this.shooterLead.getSelectedSensorVelocity();
  }
  
  public double getTDVelocity() {
    return this.shooterTD.getSelectedSensorVelocity();
  } 

  public double getPostion() {
    return this.shooterLead.getSelectedSensorPosition();
  }

  // Gets current draw of shooterLead motor
  public double getShooterCurrentDraw() {
    return this.shooterLead.getSupplyCurrent();
  }
  
  // Gets current draw of shooterTD motor
  public double getTDCurrentDraw() {
    return this.shooterTD.getSupplyCurrent();
  }

  public double getVoltageOutput() {
    return this.shooterLead.getMotorOutputVoltage();
  }
/*
  public double getShooterBackspinVelocity(){
    return this.shooterAntiBackspin.ge
  }
*/
  @Override
  public void periodic() {
    if(Robot.drivetrain.mode == DriveMode.Limelight) {

      this.setShooterSpeed((14.9 * Robot.limelight.getDistance()) + 6040); // Got this from google sheet equation, number = 4328
      //this.setShooterSpeed(8600);
		/*
			if(Robot.limelight.getDistance() <= 70) //inches 
			{
			  this.setShooterSpeed(10000);
			}
			else if(Robot.limelight.getDistance() <= 80)
			{
			  this.setShooterSpeed(10000);
			}
      else
      {
        this.setShooterSpeed(10000);
      }*/
		}
    // This method will be called once per scheduler run
  }

  public void waitForShooterSpeed(double setpoint, double threshhold){
    while(this.getShooterVelocity() > (setpoint + threshhold) || this.getShooterVelocity() < (setpoint - threshhold))
    {
      // wait for speed to be stabilized
    }
  }

  // Shooter lead and follow setpoint
  public void setShooter(ControlMode controlMode, double setpoint) {
    shooterLead.set(controlMode, setpoint);
  }

  public void setCoastMode(){
    shooterLead.setNeutralMode(NeutralMode.Coast);
  }

  // Shooter TD setpoint
  public void setShooterTD(ControlMode controlMode, double setpoint) {
    shooterTD.set(controlMode, setpoint);
  }

  // Sets the speed of the shooter motors
  public void setShooterVelocity(double setpoint) {
    this.setShooter(ControlMode.Velocity, setpoint);
  }

  public void shoot() {
    this.setShooterVelocity(this.shooterSpeed);
  }

  public void eject() {
    this.setShooterVelocity(this.shooterEjectSpeed);
  }

  public void setShooterAntiBackspin(double setpoint) {
      this.shooterAntiBackspin.set(setpoint);
  }

  public void setShooterPercentOutput(double setpoint) {
    this.setShooter(ControlMode.PercentOutput, setpoint);
  }
  // Sets the speed of the shooterTD motor
  public void setTDPercentOutput(double setpoint) {
    this.setShooterTD(ControlMode.PercentOutput, setpoint);
  }

  public void setTDVelocity(double setpoint) {
    this.setShooterTD(ControlMode.Velocity, setpoint);
  }

}
