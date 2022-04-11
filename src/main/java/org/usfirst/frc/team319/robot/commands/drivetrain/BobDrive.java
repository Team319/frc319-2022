package org.usfirst.frc.team319.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team319.models.DriveMode;
import org.usfirst.frc.team319.models.DriveSignal;
import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.utils.BobDriveHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class BobDrive extends CommandBase {

	BobDriveHelper helper;
	private double quickTurnThreshold = 0.2;

	private PIDController limelightRotatePID = new PIDController(0.25, 0.01, 0.0);

	public BobDrive() {
		addRequirements(Robot.drivetrain);
		helper = new BobDriveHelper();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		double rotateValue = 0;

		if (Robot.drivetrain.mode == DriveMode.Normal) {
			rotateValue = Robot.oi.driverController.rightStick.getX() * 0.25;
			//rotateValue = HelperFunctions.signedSquare(rotateValue);
		  }
		else if (Robot.drivetrain.mode == DriveMode.Limelight) {
			rotateValue = -limelightRotatePID.calculate(Robot.limelight.getXProportional()) ;
			//System.out.println("rotateValue = "+ rotateValue);
			System.out.println("distance = "+ Robot.limelight.getDistance());
		  }
		double moveValue = -Robot.oi.driverController.leftStick.getY() * 0.50;
		boolean quickTurn = (moveValue < quickTurnThreshold && moveValue > -quickTurnThreshold);	
		DriveSignal driveSignal = helper.cheesyDrive(moveValue, rotateValue, quickTurn, false);
		Robot.drivetrain.drive(ControlMode.PercentOutput, driveSignal);
	}

	public boolean isFinished() {
		return false;
	}

	public void end() {
	}

	public void interrupted() {
	}
}
