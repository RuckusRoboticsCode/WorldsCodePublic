package org.firstinspires.ftc.teamcode.Helper.Controllers;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HeadingPIDFController {

    private double previousError = 0.0;
    private double sumError = 0;
    private double targetPosition = 0;
    private double targetTolerance = 0.0;
    private final ElapsedTime time;

    private double kp = 0.0;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0;

    public HeadingPIDFController(PIDFCoefficients pidfCoefficients) {
        this(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.p, pidfCoefficients.f);
    }

    public HeadingPIDFController(double kp) {
        this(kp, 0, 0, 0);
    }

    public HeadingPIDFController(double kp, double ki) {
        this(kp, ki, 0, 0);
    }

    public HeadingPIDFController(double kp, double ki, double kd) {
        this(kp, ki, kd, 0);
    }

    public HeadingPIDFController(double kp, double ki, double kd, double kf) {
        this.setCoefficients(kp, ki, kd, kf);
        time = new ElapsedTime();
    }

    public void setCoefficients(double kp) {
        setCoefficients(kp, 0, 0, 0);
    }

    public void setCoefficients(double kp, double ki) {
        setCoefficients(kp, ki, 0, 0);
    }

    public void setCoefficients(double kp, double ki, double kd) {
        setCoefficients(kp, ki, kd, 0);
    }

    public void setCoefficients(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public void setTargetTolerance(double targetTolerance) {
        this.targetTolerance = targetTolerance;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = angleWrapPositive(targetPosition);
        sumError = 0;
        time.reset();
    }

    public boolean atTarget() {
        return Math.abs(previousError) < targetTolerance;
    }

    public double getPreviousError() {
        return previousError;
    }

    public double angleWrapper(double radians, boolean positiveOnly) {
        if (positiveOnly) {
            return angleWrapNegative(radians);
        }
        return angleWrapNegative(radians);
    }

    private double angleWrapPositive(double radians) {
        double modAngle = radians % (2 * Math.PI);
        if (modAngle < 0) {
            modAngle += (2 * Math.PI);
        }
        return modAngle;
    }

    private double angleWrapNegative(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        radians %= (2 * Math.PI);
        return radians;
    }

    public double update(double processVariable) {
        processVariable = angleWrapPositive(processVariable);
        double error = angleWrapper(targetPosition - processVariable, false);
        double output = kf;

        if(Math.abs(error) > targetTolerance) {
            double derivative = (error - previousError) / time.seconds();

            output = (kp * error) + (ki * sumError) + (kd * derivative) + kf;

            if (Math.abs(output) < 1) {
                sumError += error;
            }
        }

        previousError = error;
        time.reset();
        return output;
    }
}
