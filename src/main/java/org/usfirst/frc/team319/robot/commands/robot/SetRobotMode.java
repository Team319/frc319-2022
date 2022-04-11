/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot.commands.robot;

import org.usfirst.frc.team319.models.RobotMode;
//import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.Robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetRobotMode extends InstantCommand {
 private RobotMode robotMode = RobotMode.Normal;

  public SetRobotMode(RobotMode robotMode) {
    this.robotMode = robotMode;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.robotMode = this.robotMode;
  }
}
