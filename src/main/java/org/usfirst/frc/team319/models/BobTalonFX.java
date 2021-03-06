package org.usfirst.frc.team319.models;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BobTalonFX extends TalonFX implements MotorController {
    private int defaultPidIndex = 0;
    private double currentSetValue = 0;

    private int primaryPidIndex = 0;
	private int secondaryPidIndex = 1;

    public BobTalonFX(int deviceNumber) {
        super(deviceNumber);
        this.configNominalOutputForward(0.0);
        this.configNominalOutputReverse(0.0);
        this.configPeakOutputForward(1);
        this.configPeakOutputReverse(-1);
        this.configMotionProfileTrajectoryPeriod(0);
    }

    public int getDefaultPidIndex() {
        return this.defaultPidIndex;
    }

    public void setDefaultPidIndex(int pidIndex) {
        this.defaultPidIndex = pidIndex;
    }

    // ------------- HELPER METHODS -------- //

    public ErrorCode configPIDF(int slotIdx, double P, double I, double D, double F) {
        ErrorCode errorCode = ErrorCode.OK;

        errorCode = this.config_kP(slotIdx, P);
        if (errorCode != ErrorCode.OK) {
            return errorCode;
        }

        errorCode = this.config_kI(slotIdx, I);
        if (errorCode != ErrorCode.OK) {
            return errorCode;
        }

        errorCode = this.config_kD(slotIdx, D);
        if (errorCode != ErrorCode.OK) {
            return errorCode;
        }

        errorCode = this.config_kF(slotIdx, F);
        return errorCode;
    }

    public ErrorCode configPIDF(int slotIdx, double P, double I, double D, double F, int iZone) {
        ErrorCode eCode = this.configPIDF(slotIdx, P, I, D, F);
        eCode = this.config_IntegralZone(slotIdx, iZone);
        return eCode;
    }

    public ErrorCode configPIDF(PIDGains gains) {
        return this.configPIDF(gains.slot, gains.P, gains.I, gains.D, gains.F, gains.iZone);
    }

    public void configMotionParameters(MotionParameters parameters) {
        this.configMotionAcceleration(parameters.getAcceleration());
        this.configMotionCruiseVelocity(parameters.getCruiseVelocity());
        this.setGains(parameters.getGains());
    }

    public void selectMotionParameters(MotionParameters parameters) {
        this.selectProfileSlot(parameters.getGains().slot);
        this.configMotionAcceleration(parameters.getAcceleration());
        this.configMotionCruiseVelocity(parameters.getCruiseVelocity());
    }

    public ErrorCode setGains(PIDGains gains) {
        return this.configPIDF(gains.slot, gains.P, gains.I, gains.D, gains.F, gains.iZone);
    }

    public void selectProfileSlot(int slotIdx) {
        super.selectProfileSlot(slotIdx, defaultPidIndex);
    }

    public void pidWrite(double output) {
        this.currentSetValue = output;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }

    @Override
    public void set(double speed) {
        this.currentSetValue = speed;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }

    @Override
    public double get() {
        return this.currentSetValue;
    }

    @Override
    public void disable() {
        this.currentSetValue = 0;
        super.set(ControlMode.Disabled, this.currentSetValue);
    }

    @Override
    public void stopMotor() {
        this.currentSetValue = 0;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }

    public double getPrimarySensorPosition() {
		return this.getSelectedSensorPosition(primaryPidIndex);
	}

	public double getSecondarySensorPosition() {
		return this.getSelectedSensorPosition(secondaryPidIndex);
	}

	public double getPrimarySensorVelocity() {
		return this.getSelectedSensorVelocity(primaryPidIndex);
	}

	public double getSecondarySensorVelocity() {
		return this.getSelectedSensorVelocity(secondaryPidIndex);
	}
}