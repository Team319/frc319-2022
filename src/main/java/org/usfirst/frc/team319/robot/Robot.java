/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot;

import org.usfirst.frc.team319.robot.subsystems.*;
import org.usfirst.frc.team319.utils.Constants;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.usfirst.frc.team319.models.RobotMode;
import org.usfirst.frc.team319.robot.commands.auto.*;
import org.usfirst.frc.team319.robot.commands.auto.FourBallExample.FourBall;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team319.robot.commands.drivetrain.BobDrive;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends TimedRobot {

	Command autonomousCommand;

	public static Field2d field = new Field2d();
	
	public static Drivetrain drivetrain = new Drivetrain();
	public static CargoTunnel cargoTunnel = new CargoTunnel();
	public static Collector collector = new Collector();
	public static Shooter shooter = new Shooter();
	public static Climber climber = new Climber();
	public static Limelight limelight = new Limelight();
	public static OI oi;
	public static RobotMode robotMode = RobotMode.Normal;

	private Command m_teleopCommand = new BobDrive();

	//public static RobotMode mode = RobotMode.Normal;
	
	SendableChooser<Command> autoChooser;
	// SendableChooser<Command> m_chooser = new SendableChooser<>();

	@Override
	public void robotInit() {
		oi = new OI();

		robotMode = RobotMode.Normal;
		Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
		autoChooser = new SendableChooser<Command>();
		autoChooser.setDefaultOption("Shoot And Drive Forward", new JustShoot());
		autoChooser.addOption("Drive Forward", new MoveABit());
		autoChooser.addOption("2 Ball", new Basic2Ball());
		SmartDashboard.putData(autoChooser);
		autonomousCommand = autoChooser.getSelected();

		//autonomousCommand = new JustShoot();
		
		autonomousCommand = new FiveBallAuto();
		//autonomousCommand = new AutoTest();
		//autonomousCommand = new FourBallExample();

	}

	/**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
	shooter.shooterFollow.follow(shooter.shooterLead);
	climber.leftTelescope.follow(climber.rightTelescope);
	climber.leftPivot.follow(climber.rightPivot);
	drivetrain.leftFollow1.follow(drivetrain.leftLead);
	drivetrain.leftFollow2.follow(drivetrain.leftLead);
	drivetrain.rightFollow1.follow(drivetrain.rightLead);
	drivetrain.rightFollow2.follow(drivetrain.rightLead);


	//System.out.println(Robot.collector.homeSwitch.get());


		//SmartDashboard.putNumber("Collector Position", collector.getPosition());
		SmartDashboard.putNumber("Shooter Velocity", shooter.getShooterVelocity());
		//SmartDashboard.putNumber("TD Velocity", shooter.getTDVelocity());
		//SmartDashboard.putNumber("TD Current Draw", shooter.getTDCurrentDraw());
		//SmartDashboard.putNumber("Ball Tunnel Current Draw", cargoTunnel.getLeadCurrentDraw());
		//SmartDashboard.putNumber("Shooter Current Draw", shooter.getShooterCurrentDraw());
		//SmartDashboard.putNumber("Shooter Voltage", shooter.getVoltageOutput());
		//SmartDashboard.putNumber("Climber Pivot Position", climber.getRightPivotPosition());
		//SmartDashboard.putNumber("Climber Pivot Position", climber.getLeftPivotPosition());
		//SmartDashboard.putNumber("Climber Telescope Position", climber.getTelescopePosition());
		//SmartDashboard.putNumber("Collector Lead Current Draw", collector.collectorPivotLead.getOutputCurrent());
		//SmartDashboard.putNumber("Collector Follow Current Draw", collector.collectorPivotFollow.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());


		//SmartDashboard.putNumber("Amps limit changed?", collector.didCurrentLimitChange);
		//SmartDashboard.putNumber("Filtered Current", collector.filteredOutputAmps);

		SmartDashboard.putNumber("Left Drive Position", drivetrain.leftLead.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right Drive Position", drivetrain.rightLead.getSelectedSensorPosition());
		SmartDashboard.putNumber("Collector Amps", collector.collectorPivotLead.getOutputCurrent());
		
		SmartDashboard.putNumber("Odom X ", drivetrain.m_odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("Odom Y ", drivetrain.m_odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("Odom Heading ", drivetrain.m_odometry.getPoseMeters().getRotation().getDegrees());

		SmartDashboard.putData(field);

		//SmartDashboard.putBoolean("Beam Break", shooter.beamBreak.get());
		//SmartDashboard.putBoolean("Home Switch", collector.homeSwitch.get());

		//SmartDashboard.putBoolean("Velocity Reached", shooter.velocityReached);

  }

	@Override
	public void disabledInit() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Coast);

	}

	@Override
	public void disabledPeriodic() {
		CommandScheduler.getInstance().run();
		
	}

	@Override
	public void autonomousInit() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		Robot.drivetrain.resetEncoders();
		//Robot.drivetrain.resetOdometry();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		  }
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();

	}

	@Override
	public void teleopInit() {

		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		m_teleopCommand.schedule();
		
		Robot.drivetrain.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
		/*if(Robot.drivetrain.mode == DriveMode.Limelight) {

		
			if(limelight.getDistance() <= 71 || true) //inches 
			{
			  shooter.setShooterSpeed(7000);
			}
			else
			{
			  shooter.setShooterSpeed(8500);
			}
		} */
	}

	@Override
	public void testPeriodic() {
	}


	  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getTestAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.kvVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
				Constants.AutoConstants.kinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
				Constants.AutoConstants.maxSpeed,
				Constants.AutoConstants.maxAccel)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.AutoConstants.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Constants.AutoConstants.RamseteB, Constants.AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.kvVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
  */

}
