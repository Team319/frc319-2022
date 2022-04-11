// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc.team319.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.usfirst.frc.team319.utils.Constants;
import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.subsystems.Drivetrain;
import java.util.List;

public class RamsetA extends SequentialCommandGroup {

  protected Trajectory autoTrajectory;

  public static Trajectory makeTrajectory(
      double startVelocity, List<Pose2d> waypoints, double endVelocity, boolean reversed) {
    return makeTrajectory(
        startVelocity, waypoints, endVelocity, Constants.AutoConstants.maxSpeed, reversed);
  }

  public static Trajectory makeTrajectory(
      double startVelocity,
      List<Pose2d> waypoints,
      double endVelocity,
      double maxVelocity,
      boolean reversed) {
    CentripetalAccelerationConstraint centripetalAccelerationConstraint =
        new CentripetalAccelerationConstraint(Constants.AutoConstants.maxCentripetalAcceleration);
    return TrajectoryGenerator.generateTrajectory(
        waypoints,
        new TrajectoryConfig(
                Math.min(maxVelocity, Constants.AutoConstants.maxSpeed),
                Constants.AutoConstants.maxAccel)
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(reversed)
            .addConstraint(centripetalAccelerationConstraint)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                        Constants.AutoConstants.ksVolts,
                        Constants.AutoConstants.kvVoltSecondsPerMeter,
                        Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                    Constants.AutoConstants.kinematics,
                    Constants.AutoConstants.maxVoltageApplied)));
  }

  public static Command RamseteSchmoove(Trajectory autoTrajectory) {
    RamseteCommand ramsete =
        new RamseteCommand(
            autoTrajectory,
            Robot.drivetrain::getPose,
            new RamseteController(
                Constants.AutoConstants.RamseteB, Constants.AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.kvVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            Robot.drivetrain::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            Robot.drivetrain::tankDriveVolts,
            Robot.drivetrain);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              Robot.drivetrain.resetOdometry(autoTrajectory.getInitialPose());
            }),
        ramsete,
        new InstantCommand(
            () -> {
              Robot.drivetrain.tankDriveVolts(Constants.zero, Constants.zero);
              System.out.println("Trajectory done");
            }));
  }
}
