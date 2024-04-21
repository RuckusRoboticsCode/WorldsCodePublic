package org.firstinspires.ftc.teamcode.tuning.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

@TeleOp
@Config
public class DropOneTuning extends OpModeEx {

    private Servo depositLeft;
    private Servo depositRight;
    private Servo gate;

    public static double loopTimesMs = 15;
    public static double dropTimeMs = 8;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dropTimer = new ElapsedTime();

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

    @Override
    public void init() {
        super.init();
        depositLeft = hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_LEFT);
        depositRight = hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_RIGHT);
        gate = hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_RELEASE_SERVO);
    }

    @Override
    public void loop() {
        super.loop();
        if (gamepadEx1.wasJustPressed(Buttons.LEFT_BUMPER)) {
            depositLeft.setPosition(DepositPositions.OUTTAKE.leftPos);
            depositRight.setPosition(DepositPositions.OUTTAKE.rightPos);
        } else if (gamepadEx1.wasJustPressed(Buttons.RIGHT_BUMPER)) {
            gate.setPosition(GatePositions.OPEN.gatePos);
            dropTimer.reset();
        }

        if (dropTimer.milliseconds() > dropTimeMs) {
            gate.setPosition(GatePositions.CLOSED.gatePos);
        }

        while (timer.milliseconds() < loopTimesMs) {}
        timer.reset();
    }
}
