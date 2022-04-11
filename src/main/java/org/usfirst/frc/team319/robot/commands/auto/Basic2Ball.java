// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.drivetrain.DrivetrainBrakeMode;
import org.usfirst.frc.team319.robot.commands.limelight.StartLimelightMode;
import org.usfirst.frc.team319.robot.commands.limelight.StopLimelightMode;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;
import org.usfirst.frc.team319.robot.commands.shooter.ShootCommand;
import org.usfirst.frc.team319.robot.commands.shooter.StopShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Basic2Ball extends SequentialCommandGroup {
  /** Creates a new Basic2Ball. */
  public Basic2Ball() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new DrivetrainBrakeMode(NeutralMode.Brake), // Set Break Mode ( So we don't coast)
                new AutoCollectCommand(), //Drive forward ~ 5 ft @ 20 % VBUS
               // new AutoRotateToTarget(0.05), // TODO - What should this error really be?
               // new DrivetrainDriveDistance(0, -0.2), // "STOP" probably finishes instantly
                new StartLimelightMode(),
               new ShootCommand(Robot.shooter.getShooterSpeed()), // Shoot 2 balls
               new StopLimelightMode(),
               new DrivetrainBrakeMode(NeutralMode.Coast), // Stop shooter motors
                new WaitForTime(0.5), // Wait for half a second... ( probably not needed)
                new StopShooting());
                
  }
}
