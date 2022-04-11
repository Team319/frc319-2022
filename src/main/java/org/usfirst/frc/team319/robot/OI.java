 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team319.robot;

import org.usfirst.frc.team319.controllers.BobXboxController;
import org.usfirst.frc.team319.models.RobotMode;
import org.usfirst.frc.team319.robot.commands.auto.AutoRotateToTarget;
import org.usfirst.frc.team319.robot.commands.auto.AutoTest;
import org.usfirst.frc.team319.robot.commands.collector.SmartFourBar;
import org.usfirst.frc.team319.robot.commands.limelight.StartLimelightMode;
import org.usfirst.frc.team319.robot.commands.limelight.StopLimelightMode;
//import org.usfirst.frc.team319.robot.commands.collector.ResetCollectorEncoder;
import org.usfirst.frc.team319.robot.commands.robot.CollectCommand;
import org.usfirst.frc.team319.robot.commands.robot.EjectCargo;
import org.usfirst.frc.team319.robot.commands.robot.SetRobotMode;
import org.usfirst.frc.team319.robot.commands.robot.StopCollect;
import org.usfirst.frc.team319.robot.commands.robot.StopEject;
import org.usfirst.frc.team319.robot.commands.shooter.EjectFromShooter;
import org.usfirst.frc.team319.robot.commands.shooter.SetTDPercentOutput;
import org.usfirst.frc.team319.robot.commands.shooter.ShootCommand;
import org.usfirst.frc.team319.robot.commands.shooter.SmartShoot;
import org.usfirst.frc.team319.robot.commands.shooter.StopShooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public BobXboxController driverController;
	public BobXboxController operatorController;

	public OI() {
		driverController = new BobXboxController(0, 0.2, 0.2);
		operatorController = new BobXboxController(1, 0.2, 0.2);
		// ----Driver Controller---- \\
		
		driverController.rightTriggerButton.whenPressed(new ShootCommand(Robot.shooter.getShooterSpeed())); // was 14000
		driverController.rightTriggerButton.whenReleased(new StopShooting());
		
		driverController.rightBumper.whenPressed(new StopShooting());

		driverController.leftTriggerButton.whenPressed(new CollectCommand());
		driverController.leftBumper.whenPressed(new StopCollect());

		driverController.xButton.whenPressed(new EjectCargo(0.4, 0)); // just spit collector
		driverController.xButton.whenReleased(new StopEject());

		driverController.bButton.whenPressed(new StartLimelightMode());
		driverController.bButton.whenReleased(new StopLimelightMode());

		//driverController.Dpad.Down.whenPressed(new CollectorGoToHardStop(0.5));

		//driverController.bButton.whenPressed(new DummyShoot(Robot.shooter.shooterSpeed));
		
		driverController.yButton.whenPressed(new SetTDPercentOutput(-0.3));
		driverController.yButton.whenReleased(new SetTDPercentOutput(0));

		driverController.aButton.whenHeld(new SmartShoot());
		driverController.aButton.whenReleased(new SequentialCommandGroup(new StopLimelightMode(),new StopShooting()) );

		//driverController.startButton.whenPressed(new AutoRotateToTarget(0.1));

		//driverController.Dpad.Right.whileHeld(new CollectorMoveCurrent(2));
		//driverController.Dpad.Left.whileHeld(new CollectorMoveCurrent(-2));


		//driverController.bButton.whenPressed(new DrivetrainDriveDistance(100000, 0.2));

		//driverController.bButton.whenPressed(new SetShooterAntiBackspinPO(0.5));
		//driverController.bButton.whenReleased(new SetShooterAntiBackspinPO(0));

		//driverController.Dpad.Left.whileHeld(new CollectorMovePO(0.1));


		// ----Operator Controller---- \\

		//operatorController.yButton.whenPressed(new SetClimberPivotPosition(80000));
		//operatorController.aButton.whenPressed(new SetClimberPivotPosition(0));
		operatorController.startButton.whenPressed(new SetRobotMode(RobotMode.Climb));
	//	operatorController.Dpad.Up.whenPressed(new SetRobotMode(RobotMode.Normal));
		//operatorController.Dpad.Down.whenPressed(new ResetClimbEncoders());
		operatorController.rightTriggerButton.whileHeld(new EjectFromShooter(Robot.shooter.shooterEjectSpeed));
		operatorController.rightBumper.whenPressed(new StopShooting());
		
		//operatorController.rightTriggerButton.whenReleased(new StopShooting());
		operatorController.leftTriggerButton.whenPressed(new EjectCargo(0.4, -0.3));
		operatorController.leftTriggerButton.whenReleased(new StopEject());
		
		operatorController.yButton.whenPressed(new SmartFourBar(1, Robot.collector.currentLimitExtend, 1, 20));
		operatorController.aButton.whenPressed(new SmartFourBar(-1, Robot.collector.currentLimitRetract, 1, 25));

		//operatorController.xButton.whenPressed(new StartLimelightMode());
		//operatorController.xButton.whenReleased(new StopLimelightMode());

		//operatorController.Dpad.Right.whenPressed(new SetIntakeVelocity(.3));
		//operatorController.Dpad.Left.whenPressed(new SetIntakeVelocity(-.3));
		//operatorController.Dpad.Down.whenPressed(new SetIntakeVelocity(0));

		//TODO Give Nelson Smart Pivot
		//operatorController.yButton.whenPressed(new CollectorMovePO(0.2));
		//operatorController.aButton.whenPressed(new CollectorMovePO(-0.2));

	}
}
