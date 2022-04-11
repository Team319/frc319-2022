// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import java.util.List;

import org.usfirst.frc.team319.robot.commands.drivetrain.RamsetA;
import org.usfirst.frc.team319.utils.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {


  private Pose2d startPose = new Pose2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0) ,Rotation2d.fromDegrees(0));
    private Pose2d endPose = new Pose2d(Units.feetToMeters(14), Units.feetToMeters(5), Rotation2d.fromDegrees(19));
    
    private Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(startPose, endPose),
          0.0,
          Units.feetToMeters(5),
          false);


        private Trajectory leg2 =
          RamsetA.makeTrajectory(
              0.0,
              List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE),
              0.0,
              Units.feetToMeters(8),
              false);


  /** Creates a new AutoTest. */
  public AutoTest() {
    

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RamsetA.RamseteSchmoove(leg1)
    );
  }
}
