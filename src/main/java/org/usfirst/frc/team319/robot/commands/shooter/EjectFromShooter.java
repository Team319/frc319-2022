// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.robot.FeedShot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectFromShooter extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public EjectFromShooter(double shooterSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
      Order of events:  Spin up shooter, wait for shooter to spin up, feed first ball to shooter,
                        wait for first ball to be shot, slow down TD to recieve second ball, feed
                        second ball to TD, detect second ball in position, stop second ball in 
                        position, wait for shooter to get back up to speed, feed second ball to
                        shooter.
    */

    addCommands(
                new SetShooterPercentOutput(0.3), 
                new SetShooterAntiBackspinPO(Robot.shooter.antiBackspinPercent), 
                //new WaitForShooterSpeed(Robot.shooter.shooterEjectSpeed, 100),
                new FeedShot(Robot.shooter.tdShootSpeed),
                new SetShooterAntiBackspinPO(1), 
                new SetTDVelocity(Robot.shooter.tdEjectSpeed)
    );
  }
}
