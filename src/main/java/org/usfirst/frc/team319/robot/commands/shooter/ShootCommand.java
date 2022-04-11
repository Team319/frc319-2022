// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.cargoTunnel.TunnelPO;
import org.usfirst.frc.team319.robot.commands.robot.FeedShot;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommand. */
  public ShootCommand(double shooterSetpoint) {
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
                new SetShooterVelocity(shooterSetpoint), 
                new SetShooterAntiBackspinPO(Robot.shooter.antiBackspinPercent), 
                new WaitForShooterVelocity(shooterSetpoint, 0, 0.75),
                new FeedShot(Robot.shooter.tdShootSpeed),
                new SetShooterAntiBackspinPO(1), 
                new WaitForTime(0.5), 
                new SetTDVelocity(Robot.shooter.tdFeedSpeed), 
                new TunnelPO(0.2),
                new BallDetectedTD(), 
                new SetTDVelocity(0), 
                new TunnelPO(0), 
                new SetShooterVelocity(shooterSetpoint),
                //new WaitForShooterVelocity(shooterSetpoint, 0, 1.0), 
                new FeedShot(Robot.shooter.tdShootSpeed), 
                new SetShooterAntiBackspinPO(Robot.shooter.antiBackspinPercent)
               );

               System.out.println("Shoot finished");
  }
}
