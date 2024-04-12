package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;

public class Drivetrain implements Subsystem {

    private GamepadEx gamepadEx = null;
    private Telemetry telemetry = null;

    private final double multiplierFL = 1.0;
    private final double multiplierFR = 0.97895;
    private final double multiplierBL = 0.96842;
    private final double multiplierBR = 0.95789;

    private DcMotorEx FL, FR, BL, BR;

    public enum DRIVING_SPEED {
        NORMAL(1.0),
        SLOW(0.5);

        final double power;

        DRIVING_SPEED(double power) {this.power = power;}
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
        FL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.DT_FL), 0.005);
        FR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.DT_FR), 0.005);
        BL = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.DT_BL), 0.005);
        BR = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.DT_BR), 0.005);

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        if (gamepadEx != null) {

            DRIVING_SPEED speed = DRIVING_SPEED.NORMAL;
            if (gamepadEx.getRightTrigger() > 0.25) {
                speed = DRIVING_SPEED.SLOW;
            }

            double leftX = smooth(gamepadEx.getLeftX());
            double leftY = smooth(gamepadEx.getLeftY());
            double rightX = smooth(gamepadEx.getRightX());

            leftX *= 1.1;
            rightX *= 0.75;

            double sumPower = Math.abs(leftY) + Math.abs(rightX) + Math.abs(leftX);
            double denominator = Math.max(sumPower, 1.0);

            double powerFL = ((leftX + leftY + rightX) / denominator);
            double powerFR = ((leftY - leftX - rightX) / denominator);
            double powerBL = ((leftY - leftX + rightX) / denominator);
            double powerBR = ((leftY + leftX - rightX) / denominator);

            powerFL = powerFL > 0 ? powerFL * multiplierFL : powerFL / multiplierFL;
            powerFR = powerFR > 0 ? powerFR * multiplierFR : powerFR / multiplierFR;
            powerBL = powerBL > 0 ? powerBL * multiplierBL : powerBL / multiplierBL;
            powerBR = powerBR > 0 ? powerBR * multiplierBR : powerBR / multiplierBR;

            powerFL *= speed.power;
            powerFR *= speed.power;
            powerBL *= speed.power;
            powerBR *= speed.power;

            FR.setPower(powerFR);
            FL.setPower(powerFL);
            BL.setPower(powerBL);
            BR.setPower(powerBR);
        }
    }

    private double smooth(double val) {
        if (val < 0) {
            return -1.0 * val * val;
        }
        return val * val;
    }
}
