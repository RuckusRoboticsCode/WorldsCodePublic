package org.firstinspires.ftc.teamcode.tuning.custom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.Caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.Helper.Controllers.PIDFController;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

@Disabled
//@Config
@Autonomous(name="Linear Slide Tuning Auto", group="Tuning")
public class LinearSlideTuning extends OpModeEx {

    public static double loopTimesMS = 30;
    private int currentPosition = 0;
    public static int targetPosition = 0;

    private DcMotorEx leftSlide, rightSlide;
    private IntermittentVoltageSensor intermittentVoltageSensor;
    private double currentVoltage = 12.0;
    PIDFController pidfController = null;
    private ElapsedTime time = new ElapsedTime();

    public static double kp = 0.0;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kg = 0.0;

    @Override
    public void init() {
        super.init();
        leftSlide = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_LEFT));
        rightSlide = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, HardwareNames.SLIDES_RIGHT));

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

//        intermittentVoltageSensor = new IntermittentVoltageSensor(hardwareMap, 1500);
        pidfController = new PIDFController(kp, ki, kd, kg);
        time.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        currentPosition = leftSlide.getCurrentPosition();
//        currentVoltage = intermittentVoltageSensor.getVoltage();

        pidfController.setTargetPosition(targetPosition);
        pidfController.setCoefficients(kp, ki, kd, kg);
        double power = pidfController.update(currentPosition);

//        power *= (12.0 / currentVoltage);

        leftSlide.setPower(power);
        rightSlide.setPower(power);
        while (true) {
            if (!(time.milliseconds() < loopTimesMS)) break;
        }

//        telemetry.addData("voltage", currentVoltage);
        telemetry.addData("power", power);
        telemetry.addData("Current", currentPosition);
        telemetry.addData("Target", targetPosition);
        time.reset();
    }
}
