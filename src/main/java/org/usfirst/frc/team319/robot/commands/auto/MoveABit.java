// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import org.usfirst.frc.team319.robot.commands.drivetrain.DrivetrainSetPO;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveABit extends SequentialCommandGroup {
  /** Creates a new MoveABit. */
  public MoveABit() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DrivetrainSetPO(0.1), new WaitForTime(1.0), new DrivetrainSetPO(0));
  }
}
