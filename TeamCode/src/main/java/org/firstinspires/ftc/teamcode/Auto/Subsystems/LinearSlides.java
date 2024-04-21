package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.Helper.Controllers.PIDFController;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

public class LinearSlides {

    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final PIDFController controller;
    private IntermittentVoltageSensor intermittentVoltageSensor;

    public LinearSlides(LinearOpModeEx linearOpModeEx) {
        leftSlide = new CachingDcMotorEX(linearOpModeEx.hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_LEFT), 0.005);
        rightSlide = new CachingDcMotorEX(linearOpModeEx.hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_RIGHT), 0.005);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setCurrentAlert(4.25, CurrentUnit.AMPS);

//        controller = new PIDFController(0.0078 * 1.1, 0, 0.00035, 0.1);
        controller = new PIDFController(
                0.8 * org.firstinspires.ftc.teamcode.Tele.Subsystems.LinearSlides.PIDF_COEFFICIENTS.kp.teleCoefficient,
                0.8 * org.firstinspires.ftc.teamcode.Tele.Subsystems.LinearSlides.PIDF_COEFFICIENTS.ki.teleCoefficient,
                0.8 * org.firstinspires.ftc.teamcode.Tele.Subsystems.LinearSlides.PIDF_COEFFICIENTS.kd.teleCoefficient,
                org.firstinspires.ftc.teamcode.Tele.Subsystems.LinearSlides.PIDF_COEFFICIENTS.kf.teleCoefficient
        );

        controller.setTargetTolerance(20);
    }

    public void setIntermittentVoltageSensor(IntermittentVoltageSensor intermittentVoltageSensor) {
        this.intermittentVoltageSensor = intermittentVoltageSensor;
    }

    public Action moveTo(int targetPosition) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    controller.setTargetPosition(targetPosition);
                    initialized = true;
                }

                int currentPosition = leftSlide.getCurrentPosition();
                double power = controller.update(currentPosition);
                if (intermittentVoltageSensor != null) {
                    power *= (12.0 / intermittentVoltageSensor.getVoltage());
                }

                if (leftSlide.isOverCurrent() && currentPosition < 150) {
                    controller.setTargetPosition(0);
                    telemetryPacket.put("Linear Slides Target", 0);
                    telemetryPacket.put("Linear Slides Current", currentPosition);

                    leftSlide.setPower(0);
                    rightSlide.setPower(0);

                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    return false;
                }

                leftSlide.setPower(power);
                rightSlide.setPower(power);
                telemetryPacket.put("Linear Slides Target", targetPosition);
                telemetryPacket.put("Linear Slides Current", currentPosition);

                return !controller.atTarget();
            }
        };
    }
}
