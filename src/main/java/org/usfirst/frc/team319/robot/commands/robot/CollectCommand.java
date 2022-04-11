// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.robot;

import org.usfirst.frc.team319.robot.commands.cargoTunnel.DetectBallTunnel;
import org.usfirst.frc.team319.robot.commands.shooter.BallDetectedTD;
import org.usfirst.frc.team319.robot.commands.shooter.SetTDVelocity;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectCommand extends SequentialCommandGroup {
  /** Creates a new CollectCommand. */
  public CollectCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StartCollect(), new BallDetectedTD(), new SetTDVelocity(0),
                new DetectBallTunnel(20));
  }
}
