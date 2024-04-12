package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Helper.ImuControl;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Disabled
@Autonomous
public class IntermittentImuTest extends OpModeEx {

    MecanumDrive mecanumDrive;
    ImuControl imuControl;
    Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void init() {
        super.init();

        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        imuControl = new ImuControl(mecanumDrive.getLazyImu(), startingPose.heading.toDouble(), false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        mecanumDrive.updatePoseEstimate();

        telemetry.addData("3 Wheel", Math.toDegrees(mecanumDrive.getPoseEstimate().heading.toDouble()));
        telemetry.addData("IMU", Math.toDegrees(imuControl.getHeadingRad()));
    }
}
