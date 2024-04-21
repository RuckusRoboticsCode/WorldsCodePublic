package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

@Config
public class Deposit {

     public enum DepositPositions {
         INTAKE(0.19, 0.71),
         OUTTAKE(0.71, 0.19);

        final double leftPos;
        final double rightPos;

        DepositPositions(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

     public enum GatePositions {
        CLOSED(0.45),
        OPEN(0.33);

        final double gatePos;

        GatePositions(double gatePos) {
            this.gatePos = gatePos;
        }
    }

    private final Servo depositLeft;
    private final Servo depositRight;
    private final Servo gate;

    public Deposit(LinearOpModeEx linearOpModeEx) {
        depositLeft = new CachingServo(linearOpModeEx.hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_LEFT), 0.01);
        depositRight = new CachingServo(linearOpModeEx.hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_RIGHT), 0.01);
        gate = new CachingServo(linearOpModeEx.hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_RELEASE_SERVO), 0.01);

        depositLeft.setPosition(DepositPositions.INTAKE.leftPos);
        depositRight.setPosition(DepositPositions.INTAKE.rightPos);
        gate.setPosition(GatePositions.CLOSED.gatePos);
    }

    public Action moveDeposit(DepositPositions depositPosition) {
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    depositLeft.setPosition(depositPosition.leftPos);
                    depositRight.setPosition(depositPosition.rightPos);
                    initialized = true;
                }
                return false;
            }
        };
    }

    public Action flipGate(GatePositions gatePosition) {
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    gate.setPosition(gatePosition.gatePos);
                    initialized = true;
                    return true;
                }
                return false;
            }
        };
    }

//    public static double delay = 0.099;
    public Action dropOnePixel() {
        return new Action() {
            boolean initialized = false;
            final ElapsedTime timer = new ElapsedTime();
//            final double delay = 0.099;
//            final double delay = 0.0785;
//            final double delay = 0.13;
            final double delay = 0.083;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    gate.setPosition(GatePositions.OPEN.gatePos);
                    timer.reset();
                    initialized = true;
                }

                if (timer.seconds() > delay) {
                    gate.setPosition(GatePositions.CLOSED.gatePos);
                    return false;
                }
                return true;
            }
        };
    }
}
