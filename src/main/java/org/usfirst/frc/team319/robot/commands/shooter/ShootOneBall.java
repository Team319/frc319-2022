// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.robot.FeedShot;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOneBall extends SequentialCommandGroup {
  /** Creates a new ShootOneBall. */
  public ShootOneBall(double shooterSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetShooterVelocity(shooterSetpoint), 
    new SetShooterAntiBackspinPO(Robot.shooter.antiBackspinPercent), 
    new WaitForTime(0.5),
    new FeedShot(Robot.shooter.tdShootSpeed),
    new WaitForTime(0.5),
    new SetTDVelocity(0),
    new SetShooterPercentOutput(0),
    new SetShooterAntiBackspinPO(0));
  }
}
