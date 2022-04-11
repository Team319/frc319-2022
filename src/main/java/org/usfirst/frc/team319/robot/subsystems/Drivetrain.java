package org.usfirst.frc.team319.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import org.usfirst.frc.team319.models.BobTalonFX;
import org.usfirst.frc.team319.models.DriveMode;
import org.usfirst.frc.team319.models.DriveSignal;
import org.usfirst.frc.team319.models.PIDGains;
import org.usfirst.frc.team319.robot.commands.drivetrain.BobDrive;
import org.usfirst.frc.team319.utils.BobDriveHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

	// Constants / DEFINES
	public static int DRIVE_PROFILE = 0;
	public static int ROTATION_PROFILE = 1;

	// Hardware member variables

	// Right lead and follow motors
	public BobTalonFX rightLead = new BobTalonFX(1);
	public BobTalonFX rightFollow1 = new BobTalonFX(2);
	public BobTalonFX rightFollow2 = new BobTalonFX(3);

	// Left lead and follow motors
	public BobTalonFX leftLead = new BobTalonFX(4);
	public BobTalonFX leftFollow1 = new BobTalonFX(5);
	public BobTalonFX leftFollow2 = new BobTalonFX(6);

	public WPI_PigeonIMU pigeon = new WPI_PigeonIMU(21);

	public DriveMode mode = DriveMode.Normal;


	// Kinematics / Trajectory Member variables
	public DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(pigeon.getRotation2d());

	 // The robot's drive
	 private final DifferentialDrive m_drive = new DifferentialDrive(rightLead, leftLead);

	
	public double metersPerEncoderTick = 0.0000323843; 
	// 	  TODO - What is metersPerEncoderTick 
	//           
	//   2048 ticks  | 65 motor rev | 1 wheel rev           133120   ticks
	//   ____________|_____________|________________        _________    (inverted)   = 0.0000323843
	//               |             |
	//   1 motor rev | 9 wheel rev | 0.479 meter            4.311      meter
	
	// additionally, this logic wpi lib considers "pulses" a pulse refers to a full encoder cycle ( 4 edges ) we use ticks... is there a difference?
	// I think since i used the talon encoders, we only care that the distance measurement is consistent, and correct. (so meters per tick should be ok... )

	// These are example values only -  USE THE VALUES FROM CHARACTERIZATION FOR BOB!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.

    // Example value only - as above, this must be tuned for your drive!

	BobDriveHelper helper;

	public Drivetrain() {

		//odometry = new DifferentialDriveOdometry(pigeon.getRawGyro(xyz_dps));

		helper = new BobDriveHelper();

		leftLead.configFactoryDefault();
		rightLead.configFactoryDefault();

		setupSensors();
		resetEncoders();
		pigeon.reset();
		setNeutralMode(NeutralMode.Coast);

		// Left and right motors to inverted
		leftLead.setInverted(false);
		leftFollow1.setInverted(false);
		leftFollow2.setInverted(false);
		leftLead.setSensorPhase(true);

		rightLead.setInverted(true);
		rightFollow1.setInverted(true);
		rightFollow2.setInverted(true);
		rightLead.setSensorPhase(true);

		// Open loop ramp for motors
		leftLead.configOpenloopRamp(0.25);
		leftFollow1.configOpenloopRamp(0.25);
		leftFollow2.configOpenloopRamp(0.25);

		rightLead.configOpenloopRamp(0.25);
		rightFollow1.configOpenloopRamp(0.25);
		rightFollow2.configOpenloopRamp(0.25);

	}

	@Override
	public void periodic() 
	{
		
		m_odometry.update(
			Rotation2d.fromDegrees(getHeading()),
			 getLeftMotorDistanceMeters(),
			  getRightMotorDistanceMeters()
				);
	  
	}

	// vvvvvvvvvvvvvvvvvvvvvvvvvvvv Trajectory following code vvvvvvvvvvvvvvvvvvvvvvvvvvvv
	
	/**
	* Returns the currently-estimated pose of the robot.
	*
	* @return The pose.
	*/
	public Pose2d getPose() 
	{
		//System.out.println(m_odometry.getPoseMeters());
		
		return m_odometry.getPoseMeters();
	}


  	/**
  	* Returns the current wheel speeds of the robot.
  	*
  	* @return The current wheel speeds. (meters / second)
  	*/
  	public DifferentialDriveWheelSpeeds getWheelSpeeds() 
	{
    	return new DifferentialDriveWheelSpeeds(leftLead.getPrimarySensorVelocity(), rightLead.getPrimarySensorVelocity());
  	}

	/**
   	* Resets the odometry to the specified pose.
   	*
   	* @param pose The pose to which to set the odometry.
  	*/
	public void resetOdometry(Pose2d pose) 
 	 {
		resetEncoders();
		//pigeon.reset();
		m_odometry.resetPosition(pose, getPidgeonRotation2d());
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) 
	{
		leftLead.setVoltage(leftVolts);
		rightLead.setVoltage(rightVolts);
		m_drive.feed();
	}

	/**
	* Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
	*
	* @param maxOutput the maximum output to which the drive will be constrained
	*/
	public void setMaxOutput(double maxOutput) 
	{
		m_drive.setMaxOutput(maxOutput);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() 
	{
		pigeon.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	// public double getHeading() 
	// {
	// 	return pigeon.getRotation2d().getDegrees(); 
	// 		// TODO - Hey! - Verify this value is between -180 to 180 like the comment says... pretty sure WPI expects this
	// }
	/**
   	* Returns the turn rate of the robot.
   	*
  	 * @return The turn rate of the robot, in degrees per second
   	*/
  	public double getTurnRate() 
	{
    	return -pigeon.getRate();  // TODO - this minus might be wrong. Following template
  	}

	  /**
   	* Gets the average distance of the two encoders.
   	*
   	* @return the average of the two encoder readings
   	*/
 	 public double getAverageEncoderDistance() 
	  {
    	return (getLeftMotorDistanceMeters() + getRightMotorDistanceMeters()) / 2.0;
  	}

	// Helper functions

	public Rotation2d getPidgeonRotation2d()
	{
		return  pigeon.getRotation2d();
	}

	public double getHeading(){
		return Math.IEEEremainder(pigeon.getRotation2d().getDegrees(), 360)*-1;
	}

	public double getMotorVelocityMetersPerSecond(BobTalonFX motor)
	{
	  return motor.getPrimarySensorVelocity() * metersPerEncoderTick;
	}

	public double getLeftMotorDistanceMeters()
	{
		return getLeftDriveLeadDistance() * metersPerEncoderTick;
	}

	public double getRightMotorDistanceMeters()
	{
		return getRightDriveLeadDistance() * metersPerEncoderTick;
	}
	
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Trajectory following code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	

	public void setMode(DriveMode mode) {
		System.out.println("Setting the drivetrain mode to " + mode);
		this.mode = mode;
	}

	public void setupSensors() {
		/*
		 * leftLead.configPrimaryFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative)
		 * ; leftLead.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		 * 
		 * rightLead.configRemoteSensor0(leftLead.getDeviceID(),
		 * RemoteSensorSource.TalonSRX_SelectedSensor);
		 * rightLead.configSensorSum(FeedbackDevice.RemoteSensor0,
		 * FeedbackDevice.CTRE_MagEncoder_Relative);
		 * rightLead.configPrimaryFeedbackDevice(FeedbackDevice.SensorSum, 0.5);
		 * 
		 * rightLead.configRemoteSensor1(rightFollowerWithPigeon.getDeviceID(),
		 * RemoteSensorSource.GadgeteerPigeon_Yaw);
		 * rightLead.configSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor1, (3600.0
		 * / 8192.0));
		 * 
		 * rightLead.configAuxPIDPolarity(false, 0);
		 */
	}

	public void initDefaultCommand() {
		setDefaultCommand(new BobDrive()); // NOT teleopDrive()
	}

	public void configGains(PIDGains gains) {
		this.leftLead.setGains(gains);
		this.rightLead.setGains(gains);
		rightLead.configMaxIntegralAccumulator(ROTATION_PROFILE, 3000);
	}

	public void drive(ControlMode controlMode, double left, double right) {
		// Left control mode set to left
		this.leftLead.set(controlMode, left);

		// Right control mode set to right
		this.rightLead.set(controlMode, right);
	}

	public void drive(ControlMode controlMode, DriveSignal driveSignal) {
		this.drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
	}

	public double getLeftDriveLeadDistance() {
		return this.leftLead.getSelectedSensorPosition();
	}

	public double getRightDriveLeadDistance() {
		return this.rightLead.getSelectedSensorPosition();
	}

	public double getLeftDriveLeadVelocity() {
		return this.leftLead.getSelectedSensorVelocity();
	}

	public double getRightDriveLeadVelocity() {
		return this.rightLead.getSelectedSensorVelocity();
	}

	public void setDrivetrainPositionToZero() {
		this.leftLead.setSelectedSensorPosition(0);
		this.rightLead.setSelectedSensorPosition(0);
	}

	public double getLeftLeadVoltage() {
		return this.leftLead.getMotorOutputVoltage();
	}

	public double getRightLeadVoltage() {
		return this.rightLead.getMotorOutputVoltage();
	}

	public double getLeftClosedLoopError() {
		return this.leftLead.getClosedLoopError();
	}

	public double getRightClosedLoopError() {
		return this.rightLead.getClosedLoopError();
	}

	// Tells the drive train what to do when its not doing anything else
	public void setNeutralMode(NeutralMode neutralMode) {
		this.leftLead.setNeutralMode(neutralMode);
		this.rightLead.setNeutralMode(neutralMode);
	}

	/*
	 * public double getAngle() { double[] ypr = new double[3];
	 * pigeon.getYawPitchRoll(ypr); return ypr[0]; }
	 * 
	 * public void resetPigeon() { this.pigeon.setYaw(0.0, 0); // Yaw is rotation of
	 * robot during autos }
	 */
	public double getRightDistance() {
		return rightLead.getPrimarySensorPosition();
	}

	public double getLeftDistance() {
		return leftLead.getPrimarySensorPosition();
	}

	public double getVelocity() {
		return rightLead.getPrimarySensorVelocity();
	}

	public void setDrivetrain(ControlMode controlMode, Double percentOutput) {
		// Percent outpout for left motors
		this.leftLead.set(controlMode, percentOutput);
		this.leftFollow1.set(controlMode, percentOutput);
		this.leftFollow2.set(controlMode, percentOutput);

		// Percent output for right motors
		this.rightLead.set(controlMode, percentOutput);
		this.rightFollow1.set(controlMode, percentOutput);
		this.rightFollow2.set(controlMode, percentOutput);
	}

	public void resetEncoders() {
		this.leftLead.setSelectedSensorPosition(0);
		this.leftFollow1.setSelectedSensorPosition(0);
		this.leftFollow2.setSelectedSensorPosition(0);

		// Percent output for right motors
		this.rightLead.setSelectedSensorPosition(0);
		this.rightFollow1.setSelectedSensorPosition(0);
		this.rightFollow2.setSelectedSensorPosition(0);
	}



}
