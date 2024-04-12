package org.firstinspires.ftc.teamcode.Helper.Caching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachingDcMotorEX extends CachingDcMotor implements DcMotorEx{
	private final DcMotorEx motorEx;

	public CachingDcMotorEX(DcMotorEx motorEx) {
		super(motorEx);
		this.motorEx = motorEx;
	}

	public CachingDcMotorEX(DcMotorEx motorEx, double changeThreshold) {
		super(motorEx, changeThreshold);
		this.motorEx = motorEx;
	}

	@Override
	public void setMotorEnable() {
		motorEx.setMotorEnable();
	}

	@Override
	public void setMotorDisable() {
		motorEx.setMotorDisable();
	}

	@Override
	public boolean isMotorEnabled() {
		return motorEx.isMotorEnabled();
	}

	@Override
	public void setVelocity(double angularRate) {
		motorEx.setVelocity(angularRate);
	}

	@Override
	public void setVelocity(double angularRate, AngleUnit unit) {
		motorEx.setVelocity(angularRate, unit);
	}

	@Override
	public double getVelocity() {
		return motorEx.getVelocity();
	}

	@Override
	public double getVelocity(AngleUnit unit) {
		return motorEx.getVelocity(unit);
	}

	@Override
	@Deprecated
	public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
		motorEx.setPIDCoefficients(mode, pidCoefficients);
	}

	@Override
	public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
		motorEx.setPIDFCoefficients(mode, pidfCoefficients);
	}

	@Override
	public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
		motorEx.setVelocityPIDFCoefficients(p, i, d, f);
	}

	@Override
	public void setPositionPIDFCoefficients(double p) {
		motorEx.setPositionPIDFCoefficients(p);
	}

	@Override
	@Deprecated
	public PIDCoefficients getPIDCoefficients(RunMode mode) {
		return motorEx.getPIDCoefficients(mode);
	}

	@Override
	public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
		return motorEx.getPIDFCoefficients(mode);
	}

	@Override
	public void setTargetPositionTolerance(int tolerance) {
		motorEx.setTargetPositionTolerance(tolerance);
	}

	@Override
	public int getTargetPositionTolerance() {
		return motorEx.getTargetPositionTolerance();
	}
	
	@Override
	public double getCurrent(CurrentUnit unit) {
		return motorEx.getCurrent(unit);
	}

	@Override
	public double getCurrentAlert(CurrentUnit unit) {
		return motorEx.getCurrentAlert(unit);
	}

	@Override
	public void setCurrentAlert(double current, CurrentUnit unit) {
		motorEx.setCurrentAlert(current, unit);
	}

	@Override
	public boolean isOverCurrent() {
		return motorEx.isOverCurrent();
	}
}
