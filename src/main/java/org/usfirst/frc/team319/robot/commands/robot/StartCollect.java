// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.robot;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.cargoTunnel.TunnelPO;
import org.usfirst.frc.team319.robot.commands.collector.SetIntakeVelocity;
import org.usfirst.frc.team319.robot.commands.collector.SmartFourBar;
import org.usfirst.frc.team319.robot.commands.shooter.SetTDVelocity;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartCollect extends SequentialCommandGroup {
  /** Creates a new Collect. */
  boolean BALL_DETECTED = false;
  boolean BALL_NOT_DETECTED = true;

  public StartCollect() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SmartFourBar(1,Robot.collector.currentLimitExtend, 1, 20),
                new WaitForTime(0.5),
                new SetIntakeVelocity(-0.4), new TunnelPO(0.15));

    if(Robot.shooter.beamBreak.get() == BALL_NOT_DETECTED) {
      addCommands(new SetTDVelocity(Robot.shooter.tdFeedSpeed));
    }

    else if(Robot.shooter.beamBreak.get() == BALL_DETECTED) {
      addCommands(new SetTDVelocity(0));
    }

  }
}
