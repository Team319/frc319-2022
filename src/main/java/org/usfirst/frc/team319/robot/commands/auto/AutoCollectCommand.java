// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import org.usfirst.frc.team319.robot.commands.drivetrain.DrivetrainDriveDistance;
import org.usfirst.frc.team319.robot.commands.robot.CollectCommand;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollectCommand extends ParallelRaceGroup {
  /** Creates a new CollectCommand. */
  public AutoCollectCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //10,000 ticks per foot
    addCommands(new DrivetrainDriveDistance(70000, 0.2), new CollectCommand());
  }
}
