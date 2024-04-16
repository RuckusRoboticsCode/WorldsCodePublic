package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

public class Intake {

    public enum IntakeSpeeds {
        SLOW_INTAKE(0.78),
        MEDIUM_INTAKE(0.88),
        FAST_INTAKE(1.0),
        OUTTAKE(-1.0),
        STOP(0.0);

        final double power;

        IntakeSpeeds(double power) {
            this.power = power;
        }
    }

    public enum RakePositions {
        INTAKE_TWO(0.54),
        INTAKE_ONE(0.59),
        LIFTED(0.93);

        public final double position;

        RakePositions(double position) {
            this.position = position;
        }
    }

    private final DcMotorEx intakeMotor;
    private final Servo rake;
    private final ColorRangeSensor sensor;

    private RakePositions rakePosition;
    private IntakeSpeeds intakeSpeed;
    private final ElapsedTime timer;
    private IntermittentVoltageSensor intermittentVoltageSensor;

    public Intake(LinearOpModeEx linearOpModeEx) {
        intakeMotor = new CachingDcMotorEX(linearOpModeEx.hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE));
        rake = new CachingServo(linearOpModeEx.hardwareMap.get(Servo.class, HardwareNames.STACK_INTAKE), 0.01);
        sensor = linearOpModeEx.hardwareMap.get(ColorRangeSensor.class, HardwareNames.COLOR_RANGE_SENSOR);

        rakePosition = RakePositions.LIFTED;
        rake.setPosition(rakePosition.position);

        intakeSpeed = IntakeSpeeds.STOP;
        intakeMotor.setPower(intakeSpeed.power);

        timer = new ElapsedTime();
    }

    public void setIntermittentVoltageSensor(IntermittentVoltageSensor voltageSensor) {
        this.intermittentVoltageSensor = voltageSensor;
    }

    public Action moveRake(RakePositions rakePositions) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    rake.setPosition(rakePositions.position);
                    initialized = true;
                }
                rake.setPosition(rakePositions.position);
                return false;
            }
        };
    }

    public Action runIntake(IntakeSpeeds intakeSpeed, double intakeRunTime) {
        this.intakeSpeed = intakeSpeed;
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    intakeMotor.setPower(intakeSpeed.power);
                    timer.reset();
                    initialized = true;
                }
                if (intermittentVoltageSensor != null) {
                    double power = intakeSpeed.power;
                    power *= 12.0 / intermittentVoltageSensor.getVoltage();
                    intakeMotor.setPower(power);
                }

                if (timer.seconds() < intakeRunTime) {
                    return true;
                } else {
                    intakeMotor.setPower(IntakeSpeeds.STOP.power);
                    return false;
                }
            }
        };
    }

    public Action intakeTwo(IntakeSpeeds intakeSpeed, double timeout) {
        this.intakeSpeed = intakeSpeed;
        return new Action() {
            boolean initialized = false;

            boolean previouslyDetected = false;
            final double twoPixelTime = 0.175;
            final double detectedDistance = 1.0;
            final ElapsedTime sensorTimer = new ElapsedTime();
            final ElapsedTime runTimer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    intakeMotor.setPower(intakeSpeed.power);
                    initialized = true;
                    runTimer.reset();
                }

                if (intermittentVoltageSensor != null) {
                    double power = intakeSpeed.power * (12.0 / intermittentVoltageSensor.getVoltage());
                    intakeMotor.setPower(power);
                }

                boolean detected = sensor.getDistance(DistanceUnit.INCH) < detectedDistance;
//                boolean detected = false;
                if (!previouslyDetected && detected) {
                    sensorTimer.reset();
                }

                previouslyDetected = detected;

                if (runTimer.seconds() > timeout) {
                    return false;
                }
                if (sensorTimer.seconds() > twoPixelTime) {
                    return true;
                } else {
                    intakeMotor.setPower(IntakeSpeeds.STOP.power);
                    return false;
                }
            }
        };
    }
}
