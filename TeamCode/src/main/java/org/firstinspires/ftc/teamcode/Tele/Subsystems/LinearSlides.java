package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.Helper.Controllers.PIDFController;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;

public class LinearSlides implements Subsystem {

    private int currentPosition = 0;
    private int targetPosition = 0;
    private DcMotorEx leftSlide, rightSlide;
    private IntermittentVoltageSensor intermittentVoltageSensor = null;

    private double currentVoltage = 12.0;
    private PIDFController pidfController = null;
    private GamepadEx gamepadEx = null;
    private Telemetry telemetry = null;
    private boolean usePIDF = false;
    private double power = 0.0;

    private double overCurrentTime = 0.5;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public enum PIDF_COEFFICIENTS {
        kp(0.06),
        ki(0),
        kd(0.0008),
        kf(0.06);

        public final double teleCoefficient;

        PIDF_COEFFICIENTS (double teleCoefficient) {
            this.teleCoefficient = teleCoefficient;
        }
    }

    private enum SLIDE_POSITIONS {
        DOWN(0),
        LOW(900),
        MIDDLE(1450),
        HIGH(1800),
        HANGING(1450),
        LIMIT(2000);

        public final int position;

        SLIDE_POSITIONS(int position) {this.position = position;}
    }

    @Override
    public void read() {
        currentPosition = leftSlide.getCurrentPosition();
//        if (intermittentVoltageSensor != null) {
//            currentVoltage = intermittentVoltageSensor.getVoltage();
//        }
    }

    public void setVoltage(double voltage) {
        currentVoltage = voltage;
    }

    @Override
    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        pidfController.setTargetTolerance(targetPosition);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        leftSlide = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_LEFT), 0.005);
        rightSlide = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_RIGHT), 0.005);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setCurrentAlert(4.75, CurrentUnit.AMPS);

//        intermittentVoltageSensor = new IntermittentVoltageSensor(hardwareMap, 1500);

        pidfController = new PIDFController(
                PIDF_COEFFICIENTS.kp.teleCoefficient,
                PIDF_COEFFICIENTS.ki.teleCoefficient,
                PIDF_COEFFICIENTS.kd.teleCoefficient,
                PIDF_COEFFICIENTS.kf.teleCoefficient
        );

        pidfController.setTargetTolerance(15);

        read();
    }

    @Override
    public void update() {
        pidfController.setTargetPosition(targetPosition);

        if (gamepadEx == null) {
            return;
        }

        if (gamepadEx.wasJustPressed(Buttons.CROSS)) {
            setTargetPosition(SLIDE_POSITIONS.DOWN.position);
            usePIDF = true;
        } else if (gamepadEx.wasJustPressed(Buttons.SQUARE)) {
            setTargetPosition(SLIDE_POSITIONS.LOW.position);
            usePIDF = true;
        } else if (gamepadEx.wasJustPressed(Buttons.TRIANGLE)) {
            setTargetPosition(SLIDE_POSITIONS.MIDDLE.position);
            usePIDF = true;
        } else if (gamepadEx.wasJustPressed(Buttons.CIRCLE)) {
            setTargetPosition(SLIDE_POSITIONS.HIGH.position);
            usePIDF = true;
        }

        double leftY = gamepadEx.getLeftY();
        double rightY = gamepadEx.getRightY();

        if (Math.abs(leftY) > 0) {
            power = leftY;
            usePIDF = false;
        } else if (Math.abs(rightY) > 0) {
            power = rightY * 0.5;
            usePIDF = false;
        } else if (usePIDF) {
            power = pidfController.update(currentPosition);

            if (power != 0.0) {
                power *= (12.0 / currentVoltage);
            }
            if (pidfController.atTarget()) {
                usePIDF = false;
            }
        } else {
            setTargetPosition(currentPosition);
            pidfController.setTargetPosition(currentPosition);
            power = pidfController.update(currentPosition);
            if (currentPosition < 100 && targetPosition < 100) {
                power = 0.0;
            }
        }

        if (elapsedTime.seconds() < overCurrentTime && power < 0) {
            power = 0;
        }

        if (leftSlide.isOverCurrent() && currentPosition < 150) {
            elapsedTime.reset();
            power = 0;
            leftSlide.setPower(power);
            rightSlide.setPower(power);

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            setTargetPosition(0);
            usePIDF = false;
        }

        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    @Override
    public void updateTelemetry() {
        if (this.telemetry != null) {
            telemetry.addData("LS Current", leftSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LS Target Position", targetPosition);
            telemetry.addData("LS Current Position", currentPosition);
            telemetry.addData("LS Power", power);
            telemetry.addData("LS At Target", pidfController.atTarget());
            telemetry.addData("LS Using PIDF", usePIDF);
        }
    }
}
