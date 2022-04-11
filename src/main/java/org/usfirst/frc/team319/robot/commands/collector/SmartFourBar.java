// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.collector;

import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SmartFourBar extends CommandBase {


  double averagedCurrent = 0;
  double setpoint = 0;

  int initCurrentLimit = 0;
  int newCurrentLimit = 0;

  double leadCurrent = 0;
  double currentThreshold = 0;

  /** Creates a new RetractFourBarSmart. */
  public SmartFourBar(double setpoint_, int initCurrentLimit_, int newCurrentLimit_, double currentThreshold_) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.collector);
    this.setpoint = setpoint_;
    this.initCurrentLimit = initCurrentLimit_;
    this.newCurrentLimit = newCurrentLimit_;
    this.currentThreshold = currentThreshold_;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Robot.collector.collectorPivotLead.setSmartCurrentLimit(this.initCurrentLimit);
    Robot.collector.collectorPivotFollow.setSmartCurrentLimit(this.initCurrentLimit);
    Robot.collector.didCurrentLimitChange++;
    Robot.collector.setCollectorPivot(this.setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.leadCurrent = Robot.collector.collectorPivotLead.getOutputCurrent();
    //double followCurrent = Robot.collector.collectorPivotFollow.getOutputCurrent();
    //this.averagedCurrent = (leadCurrent + followCurrent)/2;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.collector.collectorPivotLead.setSmartCurrentLimit(this.newCurrentLimit);
    Robot.collector.collectorPivotFollow.setSmartCurrentLimit(this.newCurrentLimit);
  
    System.out.println("Changed SmartCurrentLimits");
    Robot.collector.didCurrentLimitChange++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Robot.collector.filteredOutputAmps > this.currentThreshold);
  }
}
