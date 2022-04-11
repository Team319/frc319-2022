/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot.commands.limelight;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team319.models.DriveMode;
import org.usfirst.frc.team319.robot.commands.drivetrain.DrivetrainBrakeMode;
import org.usfirst.frc.team319.robot.commands.robot.SetDriveMode;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StopLimelightMode extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public StopLimelightMode() {
    addCommands(new SetDriveMode(DriveMode.Normal),
               new TurnLedOff());

  }
}
