package org.usfirst.frc.team319.robot.commands.drivetrain;

import org.usfirst.frc.team319.robot.Robot;
import org.usfirst.frc.team319.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class DrivetrainGoToSpeed extends CommandBase {

	public DrivetrainGoToSpeed() {
		addRequirements(Robot.drivetrain);
	}

	public void initialize() {
		Robot.drivetrain.leftLead.selectProfileSlot(Drivetrain.DRIVE_PROFILE, 0);
		Robot.drivetrain.rightLead.selectProfileSlot(Drivetrain.DRIVE_PROFILE, 0);
	}

	public void execute() {
		Robot.drivetrain.drive(ControlMode.Velocity, 3244, 3244);
		System.out.println("Left Error: " + Robot.drivetrain.getLeftClosedLoopError() + "Right Error: "
				+ Robot.drivetrain.getRightClosedLoopError());
	}

	public boolean isFinished() {
		return false;
	}

}
