// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.shooter;
import org.usfirst.frc.team319.robot.Robot;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTDPercentOutput extends InstantCommand {
  double percentage;
  public SetTDPercentOutput(double p) {
    /** Creates a new SetShooterVelocity. */
      addRequirements(Robot.shooter);
  
      percentage = p;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.setTDPercentOutput(percentage);
  }
}
