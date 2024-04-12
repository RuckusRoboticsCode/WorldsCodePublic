package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.cAprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Disabled
@Autonomous
public class WaitForRobot extends LinearOpModeEx {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        aprilTagProcessor = new cAprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setTagFamily(cAprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(Camera.fx, Camera.fy, Camera.cx, Camera.cy)
                .build();

        aprilTagProcessor.setDecimation(1);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .addProcessors(aprilTagProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(aprilTagProcessor, 0);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(waitForBackdropClear());
        }
    }

    public Action waitForBackdropClear() {
        return new Action() {
            final double buffer = 1.5;
            final int normalNumAprilTags = 3;
            final ElapsedTime timer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();
                boolean backdropClear;
                if (detections != null) {
                    telemetryPacket.put("Null?", "No");
                    telemetryPacket.put("# April Tags Detected", detections.size());
                    backdropClear = (detections.size() == normalNumAprilTags);
                } else {
                    telemetryPacket.put("Null?", "Yes");
                    backdropClear = false;
                }

                if (!backdropClear) {
                    timer.reset();
                }

                telemetryPacket.put("Time", timer.seconds());
                telemetryPacket.put("Backdrop Clear", backdropClear);
                return timer.seconds() < buffer;
            }
        };
    }
}
