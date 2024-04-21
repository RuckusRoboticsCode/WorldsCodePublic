package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Helper.BooleanEdgeDetector;

@Config
public class Deposit implements Subsystem {

    private enum DepositPositions {
//        INTAKE(0.21, 0.73),
        INTAKE(0.19, 0.71),
//        OUTTAKE(0.74, 0.22);
        OUTTAKE(0.71, 0.19);

        final double leftPos;
        final double rightPos;

        DepositPositions(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    private enum GatePositions {
        CLOSED(0.45),
        OPEN(0.33);

        final double gatePos;

        GatePositions(double gatePos) {
            this.gatePos = gatePos;
        }
    }

    private GamepadEx gamepadEx1 = null;
    private GamepadEx gamepadEx2 = null;
    private Telemetry telemetry = null;
    private Servo depositLeft = null;
    private Servo depositRight = null;
    private Servo gate = null;
    private ColorRangeSensor colorRangeSensor = null;

//    private int numDetections = 0;
    private BooleanEdgeDetector sensorDetector = null;
    private BooleanEdgeDetector triggerDetector = null;
    private BooleanEdgeDetector shouldOuttakeDetector = null;
    private double closeGateTime = 1.5;
    private ElapsedTime closeGateTimer = null;
    private BooleanEdgeDetector closeGateDetector = null;
//    private EdgeDetector numPixelsDetector = null;

    private final double DETECTED_THRESHOLD = 1.0;
//    public static double DETECTED_TIME = 0.25;
    public static double DETECTED_TIME = 0.2;
//    public static double ONE_PIXEL_TIME = 0.099;
    public static double ONE_PIXEL_TIME = 0.08;
//    public static double ONE_PIXEL_TIME = 0.128;
    private ElapsedTime timer = null;
    private ElapsedTime outtakeTimer = null;
    private ElapsedTime detectionTimer = null;

    private boolean readSensor = true;
    private boolean isDepositingOne = false;
    private boolean shouldOuttake = false;
    private double lastTime = 0.0;

    private GatePositions gatePosition = GatePositions.CLOSED;
    private DepositPositions depositPosition = DepositPositions.INTAKE;

    @Override
    public void read() {
        updateColorSensor();
    }

    @Override
    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx2 = gamepadEx;
    }

    public void setGamepadExs(GamepadEx gamepadEx1, GamepadEx gamepadEx2) {
        this.gamepadEx1 = gamepadEx1;
        this.gamepadEx2 = gamepadEx2;
        triggerDetector = new BooleanEdgeDetector(() -> Math.abs(this.gamepadEx1.getLeftTrigger()) > 0.5);
    }

    @Override
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, HardwareNames.COLOR_RANGE_SENSOR);
        gate = new CachingServo(hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_RELEASE_SERVO));
        depositLeft = new CachingServo(hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_LEFT));
        depositRight = new CachingServo(hardwareMap.get(Servo.class, HardwareNames.DEPOSIT_SERVO_RIGHT));

        sensorDetector = new BooleanEdgeDetector(() -> colorRangeSensor.getDistance(DistanceUnit.INCH) < DETECTED_THRESHOLD);
        if (gamepadEx1 != null) {
            triggerDetector = new BooleanEdgeDetector(() -> Math.abs(gamepadEx1.getLeftTrigger()) > 0.5);
        } else {
            triggerDetector = new BooleanEdgeDetector(() -> Math.abs(gamepadEx2.getLeftTrigger()) > 0.5);
        }
        shouldOuttakeDetector = new BooleanEdgeDetector(this::getShouldOuttake);
        closeGateTimer = new ElapsedTime();
        closeGateDetector = new BooleanEdgeDetector(() -> closeGateTimer.seconds() > closeGateTime);
//        numPixelsDetector = new EdgeDetector(() -> getNumDetections() == 2);
        timer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        detectionTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        sensorDetector.update();
        closeGateDetector.update();
        triggerDetector.update();

        if (gamepadEx2 != null && gamepadEx1 != null) {

            if (gamepadEx1 != null) {
                if (gamepadEx1.wasJustPressed(Buttons.LEFT_BUMPER)) {
                    gatePosition = GatePositions.OPEN;
                    closeGateTimer.reset();
                }
                if (gamepadEx2.wasJustPressed(Buttons.LEFT_BUMPER)) {
                    gatePosition = GatePositions.CLOSED;
                }
            } else {
                if (gamepadEx2.wasJustPressed(Buttons.LEFT_BUMPER)) {
                    gatePosition = GatePositions.OPEN;
                    closeGateTimer.reset();
                }
            }

            if (gamepadEx2.wasJustPressed(Buttons.RIGHT_BUMPER)) {
                if (depositPosition == DepositPositions.INTAKE) {
                    depositPosition = DepositPositions.OUTTAKE;
                } else {
                    depositPosition = DepositPositions.INTAKE;
                }
            }

            if (triggerDetector.wasJustTriggered()) {
                gatePosition = GatePositions.OPEN;
                timer.reset();
                isDepositingOne = true;
            } else if (isDepositingOne && timer.seconds() > ONE_PIXEL_TIME) {
                gatePosition = GatePositions.CLOSED;
                isDepositingOne = false;
            }

            if (gatePosition == GatePositions.OPEN) {
                gamepadEx2.getGamepad().rumble(50);
            } else {
                gamepadEx2.getGamepad().stopRumble();
            }
        }

        if (closeGateDetector.wasJustTriggered()) {
            gatePosition = GatePositions.CLOSED;
        }

        gate.setPosition(gatePosition.gatePos);
        depositLeft.setPosition(depositPosition.leftPos);
        depositRight.setPosition(depositPosition.rightPos);
        lastTime = closeGateTimer.seconds();

        if (telemetry != null) {
            telemetry.addData("Should Outtake", shouldOuttake);
            telemetry.addData("Sensor Just Triggered", sensorDetector.wasJustTriggered());
        }
    }

    private void updateColorSensor() {
        shouldOuttakeDetector.update();
        if (!readSensor) {
            return;
        }

        sensorDetector.update();
        if (!sensorDetector.getCurrentValue()) {
            detectionTimer.reset();
            shouldOuttake = false;
        }

        if (detectionTimer.seconds() > DETECTED_TIME && depositPosition != DepositPositions.OUTTAKE) {
            shouldOuttake = true;
            if (gamepadEx1 != null) {
                gamepadEx1.getGamepad().rumble(50);
            }
        } else {
            shouldOuttake = false;
        }

        if (shouldOuttakeDetector.wasJustTriggered()) {
            outtakeTimer.reset();
        }

        if (gamepadEx1 != null) {
            if (outtakeTimer.seconds() <= 0.25) {
                gamepadEx1.getGamepad().rumble(500);
            } else {
                gamepadEx1.getGamepad().stopRumble();
            }
        }
    }

    public boolean getShouldOuttake() {
        return shouldOuttake;
    }
}
