package org.firstinspires.ftc.teamcode.Helper.Caching;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CachingDcMotor extends CachingDcMotorSimple implements DcMotor {
	private final DcMotor dcMotor;
	private double cachedTargetPosition;

	public CachingDcMotor(DcMotor motor) {
		super(motor);
		cachedTargetPosition = 0.0;
		this.dcMotor = motor;
	}

	public CachingDcMotor(DcMotor motor, double changeThreshold) {
		super(motor, changeThreshold);
		cachedTargetPosition = 0.0;
		this.dcMotor = motor;
	}

	@Override
	public MotorConfigurationType getMotorType() {
		return dcMotor.getMotorType();
	}

	@Override
	public void setMotorType(MotorConfigurationType motorType) {
		dcMotor.setMotorType(motorType);
	}

	@Override
	public DcMotorController getController() {
		return dcMotor.getController();
	}

	@Override
	public int getPortNumber() {
		return dcMotor.getPortNumber();
	}

	@Override
	public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
		dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
	}

	@Override
	public ZeroPowerBehavior getZeroPowerBehavior() {
		return dcMotor.getZeroPowerBehavior();
	}

	@Override
	@Deprecated
	public void setPowerFloat() {
		dcMotor.setPowerFloat();
	}

	@Override
	@Deprecated
	public boolean getPowerFloat() {
		return dcMotor.getPowerFloat();
	}

	@Override
	public void setTargetPosition(int position) {
		if(Math.abs(cachedTargetPosition - position) >= changeThreshold) {
			dcMotor.setTargetPosition(position);
			cachedTargetPosition = position;
		}
	}

	@Override
	public int getTargetPosition() {
		return dcMotor.getTargetPosition();
	}

	@Override
	public boolean isBusy() {
		return dcMotor.isBusy();
	}

	@Override
	public int getCurrentPosition() {
		return dcMotor.getCurrentPosition();
	}

	@Override
	public void setMode(RunMode mode) {
		dcMotor.setMode(mode);
	}

	@Override
	public RunMode getMode() {
		return dcMotor.getMode();
	}
}
