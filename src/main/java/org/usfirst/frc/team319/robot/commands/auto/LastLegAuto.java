// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.auto;

import java.lang.reflect.Field;
import java.util.List;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.commands.drivetrain.RamsetA;
import org.usfirst.frc.team319.robot.commands.limelight.StartLimelightMode;
import org.usfirst.frc.team319.robot.commands.limelight.StopLimelightMode;
import org.usfirst.frc.team319.robot.commands.robot.CollectCommand;
import org.usfirst.frc.team319.robot.commands.robot.SetDriveMode;
import org.usfirst.frc.team319.robot.commands.robot.StartCollect;
import org.usfirst.frc.team319.robot.commands.robot.WaitForTime;
import org.usfirst.frc.team319.robot.commands.shooter.ShootCommand;
import org.usfirst.frc.team319.robot.commands.shooter.ShootOneBall;
import org.usfirst.frc.team319.robot.commands.shooter.StopShooting;
import org.usfirst.frc.team319.utils.FieldConstants;
import org.usfirst.frc.team319.utils.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LastLegAuto extends SequentialCommandGroup {
  
  private Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(230);
  private Rotation2d cargoGDangleOfApproach = Rotation2d.fromDegrees(-90);

  private Pose2d shot1AimPose =
  new Pose2d(FieldConstants.cargoE.getTranslation(), Rotation2d.fromDegrees(-100));

  private Pose2d prepPose =
  new Pose2d(FieldConstants.StartingPoints.tarmacD.getTranslation(), Rotation2d.fromDegrees(-100));
  

  private Pose2d cargoDPose =
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach)
          .transformBy(Util.Geometry.transformFromTranslation(0, Units.inchesToMeters(24)));

  private Pose2d cargoGPose =
      new Pose2d(FieldConstants.cargoG.getTranslation(), cargoGDangleOfApproach);


  private Pose2d startPose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0) ,Rotation2d.fromDegrees(0));
  private Pose2d Ball4Pose = new Pose2d(Units.feetToMeters(8), Units.feetToMeters(4), Rotation2d.fromDegrees(-30));
  private Pose2d Rotate1Pose = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(-60));
  private Pose2d Ball2Pose = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(-7), Rotation2d.fromDegrees(0));
  private Pose2d Shot2Pose = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(120));

    private Trajectory GoToBall4Pose =
      RamsetA.makeTrajectory(
          0.0,
          List.of(startPose, Ball4Pose),
          0.0,
          Units.feetToMeters(6), // 5 works GREAT
          false);

    private Trajectory rotateInPlace =
      RamsetA.makeTrajectory(
          0.0,
          List.of(FieldConstants.cargoE, prepPose),
          0.0,
          Units.feetToMeters(6), // 5 works GREAT
          true); 

    

  /** Creates a new AutoTest. */
  public LastLegAuto() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      RamsetA.RamseteSchmoove(GoToBall4Pose)

      //RamsetA.RamseteSchmoove(rotateToShot2)
    );
    
    //Robot.field.getObject("reference").setTrajectory(rotateToShot2);
    

  }
}
