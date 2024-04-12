package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Helper.BooleanEdgeDetector;

public class Intake implements Subsystem {

    private enum IntakeSpeeds {
        SLOW_INTAKE(0.78),
        MEDIUM_INTAKE(0.8),
        FAST_INTAKE(1.0),
        OUTTAKE(-1.0);

        final double power;

        IntakeSpeeds(double power) {
            this.power = power;
        }
    }

    enum RakePositions {
        INTAKE_TWO(0.54),
        INTAKE_ONE(0.59),
        LIFTED(0.93);

        public final double position;

        RakePositions(double position) {
            this.position = position;
        }
    }

    private GamepadEx gamepadEx = null;
    private Telemetry telemetry = null;
    private DcMotorEx intakeMotor = null;
    private Servo rake = null;
    private RakePositions rakePositions = RakePositions.LIFTED;

    private boolean shouldOuttake = false;
    private BooleanEdgeDetector outtakeDetector = null;

    private ElapsedTime timer = null;
    private final double INTAKE_TIME = 0.1;
    private final double OUTTAKE_TIME = 0.6;

    private double currentVoltage = 12.0;

    public void setVoltage(double voltage) {
        this.currentVoltage = voltage;
    }

    public void read(Deposit deposit) {
        shouldOuttake = deposit.getShouldOuttake();
    }

    @Override
    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        intakeMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE), 0.01);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rake = new CachingServo(hardwareMap.get(Servo.class, HardwareNames.STACK_INTAKE));
        rakePositions = RakePositions.LIFTED;
        rake.setPosition(rakePositions.position);

        shouldOuttake = false;
        outtakeDetector = new BooleanEdgeDetector(this::getShouldOuttake);
        outtakeDetector.update();
        timer = new ElapsedTime();
    }

    @Override
    public void update() {
        double power = 0.0;
        outtakeDetector.update();
//        if (gamepadEx.getRightTrigger() > 0.5) {
//            power = IntakeSpeeds.SLOW_INTAKE.power;
//        } else if (gamepadEx.getRightTrigger() != 0) {
//            power = IntakeSpeeds.MEDIUM_INTAKE.power;
//        } else if (gamepadEx.getLeftTrigger() > 0.5) {
//            power = IntakeSpeeds.OUTTAKE.power;
//        } else if (outtakeDetector.wasJustTriggered()) {
//            timer.reset();
//        } else if (timer.seconds() < INTAKE_TIME) {
//            power = IntakeSpeeds.FAST_INTAKE.power;
//        } else if (timer.seconds() < OUTTAKE_TIME) {
//            power = IntakeSpeeds.OUTTAKE.power;
//        }

        if (gamepadEx.getButton(Buttons.RIGHT_BUMPER) && !outtakeDetector.getCurrentValue()) {
            power = IntakeSpeeds.MEDIUM_INTAKE.power;
            power *= (12.0 / currentVoltage);
        } else if (gamepadEx.getButton(Buttons.SQUARE) && !outtakeDetector.getCurrentValue()) {
            power = IntakeSpeeds.OUTTAKE.power;
        } else if (outtakeDetector.wasJustTriggered()) {
            timer.reset();
        } else if (timer.seconds() < INTAKE_TIME) {
            power = IntakeSpeeds.FAST_INTAKE.power;
        } else if (timer.seconds() < OUTTAKE_TIME) {
            power = IntakeSpeeds.OUTTAKE.power;
        }

        if (gamepadEx.wasJustPressed(Buttons.CROSS)) {
            if (rakePositions == RakePositions.LIFTED) {
                rakePositions = RakePositions.INTAKE_TWO;
            } else {
                rakePositions = RakePositions.LIFTED;
            }
        } else if (gamepadEx.wasJustPressed(Buttons.CIRCLE)) {
            rakePositions = RakePositions.INTAKE_ONE;
        }

        rake.setPosition(rakePositions.position);
        intakeMotor.setPower(power);
    }

    private boolean getShouldOuttake() {
        return shouldOuttake;
    }
}
