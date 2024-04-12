package org.firstinspires.ftc.teamcode.tuning.custom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.Controllers.HeadingPIDFController;
import org.firstinspires.ftc.teamcode.Helper.Controllers.PIDFController;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

@Config
@Autonomous
@Disabled
public class P2pTuning extends OpModeEx {

    ThreeDeadWheelLocalizer localizer;
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeadingDeg = 0;
    private Pose2d pose = new Pose2d(0, 0, 0);

    HeadingPIDFController headingController = new HeadingPIDFController(0, 0, 0);
    PIDFController axialController = new PIDFController(0, 0, 0);
    PIDFController lateralController = new PIDFController(0, 0, 0);

    public static double headingKp = -0.7;
    public static double headingKd = 0.00002;

    public static double axialKp = 0.1;
    public static double axialKd = 0.00002;

    public static double lateralKp = 0.16;
    public static double lateralKd = 0.00002;

    DcMotorEx leftFront, leftBack, rightBack, rightFront;

    @Override
    public void init() {
        super.init();

        localizer = new ThreeDeadWheelLocalizer(
                hardwareMap,
                MecanumDrive.PARAMS.inPerTick
        );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, HardwareNames.DT_FL);
        leftBack = hardwareMap.get(DcMotorEx.class, HardwareNames.DT_BL);
        rightBack = hardwareMap.get(DcMotorEx.class, HardwareNames.DT_BR);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareNames.DT_FR);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        Pose2d targetPose = new Pose2d(targetX, targetY, Math.toRadians(targetHeadingDeg));
        headingController.setCoefficients(headingKp, 0, headingKd);
        axialController.setCoefficients(axialKp, 0, axialKd);
        lateralController.setCoefficients(lateralKp, 0, lateralKd);

        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());
        double heading = pose.heading.toDouble();

        double xError = targetPose.position.x - pose.position.x;
        double yError = targetPose.position.y - pose.position.y;

        double axialError = xError * Math.cos(heading) + yError * Math.sin(heading);
        double lateralError = xError * Math.sin(heading) - yError * Math.cos(heading);

        telemetry.addData("Axial Error", axialError);
        telemetry.addData("Lateral Error", lateralError);
        telemetry.addData("Heading Error", Math.toDegrees(headingController.getPreviousError()));

        headingController.setTargetPosition(targetPose.heading.toDouble());
        axialController.setTargetPosition(axialError);
        lateralController.setTargetPosition(lateralError);

        double headingPower = headingController.update(heading);
        double axialPower = axialController.update(0);
        double lateralPower = lateralController.update(0);

        double denominator = Math.max(Math.abs(headingPower) + Math.abs(axialPower) + Math.abs(lateralPower), 1.0);

        leftFront.setPower((axialPower + lateralPower + headingPower) / denominator);
        leftBack.setPower((axialPower - lateralPower + headingPower) / denominator);
        rightFront.setPower((axialPower - lateralPower - headingPower) / denominator);
        rightBack.setPower((axialPower + lateralPower - headingPower) / denominator);
    }
}
