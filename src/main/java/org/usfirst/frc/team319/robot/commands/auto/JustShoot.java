

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.drivetrain.DrivetrainDriveDistance;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;
import org.usfirst.frc.team319.robot.commands.shooter.ShootOneBall;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustShoot extends SequentialCommandGroup {
  /** Creates a new JustShoot. */
  public JustShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootOneBall(6000), new WaitForTime(2.0), new DrivetrainDriveDistance(60000,0.3));
  }
}
