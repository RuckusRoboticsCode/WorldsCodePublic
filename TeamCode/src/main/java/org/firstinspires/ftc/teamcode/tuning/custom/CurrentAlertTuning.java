package org.firstinspires.ftc.teamcode.tuning.custom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

//@Config
@Disabled
@TeleOp(name="Current Alert Tuning", group="Tuning")
public class CurrentAlertTuning extends OpModeEx {

    DcMotorEx slideLeft;
    DcMotorEx slideRight;
    public static double currentAlert = 4.0;

    @Override
    public void init() {
        super.init();

        slideLeft = hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_LEFT);
        slideRight = hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_RIGHT);

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setCurrentAlert(currentAlert, CurrentUnit.AMPS);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        slideLeft.setCurrentAlert(currentAlert, CurrentUnit.AMPS);

        double power = gamepadEx1.getLeftY();
        slideLeft.setPower(power);
        slideRight.setPower(power);

        telemetry.addData("Current", slideLeft.getCurrent(CurrentUnit.AMPS));

        if (slideLeft.isOverCurrent()) {
            telemetry.addData("Over Current", "Yes");
        } else {
            telemetry.addData("Over Current", "No");
        }
    }
}
