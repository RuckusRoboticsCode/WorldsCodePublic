package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Helper.ImuControl;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.cAprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class AprilTagDisplacementTest extends LinearOpModeEx {

    AprilTagProcessor processor;
    VisionPortal portal;
    ImuControl imuControl;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

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

        imuControl = new ImuControl(
                new LazyImu(hardwareMap, HardwareNames.IMU,
                        new RevHubOrientationOnRobot(MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection)),
                Math.toRadians(180)
        );

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket p = new TelemetryPacket();
            ArrayList<AprilTagDetection> detections = processor.getDetections();
            ArrayList<Vector2d> fromCamera = new ArrayList<>();
            ArrayList<Vector2d> rc = new ArrayList<>();
            ArrayList<Vector2d> fc = new ArrayList<>();

            for (AprilTagDetection det : detections) {
                fromCamera.add(
                        new Vector2d(det.ftcPose.y, det.ftcPose.x)
                );

                rc.add(
                        new Vector2d(det.ftcPose.y - Camera.forwardOffset, det.ftcPose.x - Camera.lateralOffset)
                );
            }

            p.put("From Camera", fromCamera);
            p.put("Robot Centric", rc);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    public Vector2d averageVectors (ArrayList<Vector2d> vectors) {
        Vector2d avg = new Vector2d(0, 0);
        for (Vector2d vec : vectors) {
            avg = avg.plus(vec);
        }
        avg = avg.div(vectors.size());
        return avg;
    }
}
