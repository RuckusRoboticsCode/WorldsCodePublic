package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagPoseSolver;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.cAprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Disabled
@Config
@Autonomous(name="atag Pose Test", group="Test")
public class AprilTagPoseTest extends LinearOpModeEx {

    AprilTagProcessor processor;
    VisionPortal portal;
    Pose2d pose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(
                hardwareMap, MecanumDrive.PARAMS.inPerTick
        );

        processor = new cAprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setTagFamily(cAprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(Camera.fx, Camera.fy, Camera.cx, Camera.cy)
                .build();
        processor.setDecimation(1);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(processor)
                .build();

        portal.setProcessorEnabled(processor, true);

        FtcDashboard.getInstance().startCameraStream(processor, 0);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket p = new TelemetryPacket();
            Canvas c = p.fieldOverlay();

//            pose = pose.plus(localizer.update().value());
            pose = new Pose2d(0, 0, Math.toRadians(180));
            ArrayList<AprilTagDetection> detections = processor.getDetections();
            Pose2d robotPose = AprilTagPoseSolver.getRobotPoseBackdrop(
                    detections,
                    pose
            );

//            Drawing.drawRobot(c, pose);
            Drawing.drawRobot(c, robotPose);

//            p.put("Atag Pose", robotPose);
//            p.put("Localizer Pose", pose);
            p.put("X", robotPose.position.x);
            p.put("Y", robotPose.position.y);
            p.put("Heading", Math.toDegrees(robotPose.heading.toDouble()));

//            telemetry.addData("Atag Pose", robotPose);
//            telemetry.addData("Localizer Pose", pose);

            telemetry.addData("# det", detections.size());
            telemetry.addData("X", robotPose.position.x);
            telemetry.addData("Y", robotPose.position.y);
            telemetry.addData("Heading", Math.toDegrees(robotPose.heading.toDouble()));

            FtcDashboard.getInstance().sendTelemetryPacket(p);
            telemetry.update();
        }
    }
}
