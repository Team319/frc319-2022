// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import java.util.List;

import org.usfirst.frc.team319.robot.commands.drivetrain.RamsetA;
import org.usfirst.frc.team319.robot.commands.limelight.StartLimelightMode;
import org.usfirst.frc.team319.robot.commands.robot.CollectCommand;
import org.usfirst.frc.team319.robot.commands.shooter.ShootCommand;
import org.usfirst.frc.team319.utils.FieldConstants;
import org.usfirst.frc.team319.utils.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallExample extends SequentialCommandGroup {
  /** Creates a new FourBallExample. */
 
public class FourBall extends SequentialCommandGroup {
  private Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(180);
  private Pose2d cargoDPose =
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach)
          .transformBy(Util.Geometry.transformFromTranslation(0, -Units.inchesToMeters(13)));

  private Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.StartingPoints.tarmacD, FieldConstants.cargoE),
          0.0,
          Units.feetToMeters(8),
          false);

  private Trajectory leg2 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.cargoE, FieldConstants.StartingPoints.fenderB),
          0.0,
          Units.feetToMeters(5),
          true);

  private Trajectory leg3 =
      RamsetA.makeTrajectory(
          0,
          List.of(
              FieldConstants.StartingPoints.fenderB,
              cargoDPose,
              FieldConstants.cargoG.transformBy(
                  Util.Geometry.transformFromTranslation(
                      -Units.inchesToMeters(18), -Units.inchesToMeters(15)))),
          0,
          false);

  private Trajectory leg4 =
      RamsetA.makeTrajectory(
          0,
          List.of(
              FieldConstants.cargoG,
              FieldConstants.StartingPoints.fenderB.transformBy(
                  Util.Geometry.transformFromTranslation(
                      -Units.inchesToMeters(22),
                      -Units.inchesToMeters(10)))), // 8-13 is probably acceptable
          0,
          true);

 /* public static Command scoreAllBalls(SnekSystem snekSystem, ShootSubsystem shootSubsystem) {
    return new FinishShot(snekSystem, shootSubsystem);
  }*/

  public FourBall() {

    Command driveToFirstBallAndPickUp =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg1)
            //new CollectCommand()
            );

    Command driveToHubFromFirstBall =
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new StartLimelightMode(),
                RamsetA.RamseteSchmoove(leg2)),
            new CollectCommand());

    Command driveThroughThirdBallToFourth =
        new ParallelDeadlineGroup(
            RamsetA.RamseteSchmoove(leg3),
            new CollectCommand()
           // new ShootCommand(1)
           );

    Command driveToHubAgain =
        new ParallelRaceGroup(
               // new ShootCommand(1),
              RamsetA.RamseteSchmoove(leg4),
            new CollectCommand());

    addCommands(
        driveToFirstBallAndPickUp,
        driveToHubFromFirstBall,
        new ShootCommand(1),
        driveThroughThirdBallToFourth,
        driveToHubAgain,
        new ShootCommand(1));
    }
  }
}
