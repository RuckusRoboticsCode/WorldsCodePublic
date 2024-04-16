package org.firstinspires.ftc.teamcode.Auto.Subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagPoseSolver;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.PixelLocation;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.cAprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Vision.Prop.LeftPropProcessor;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;
import org.firstinspires.ftc.teamcode.Vision.Prop.PropProcessor;
import org.firstinspires.ftc.teamcode.Vision.Prop.RightPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Vision {

    private final VisionPortal portal;
    private PropProcessor propProcessor;
    private AprilTagProcessor aprilTagProcessor;

    public Vision(LinearOpModeEx linearOpModeEx, Util.AllianceColor allianceColor, boolean startingClose) {

        setupPropProcessor(linearOpModeEx, allianceColor, startingClose);
        setupAprilTagProcessor(linearOpModeEx);

        portal = new VisionPortal.Builder()
                .setCamera(linearOpModeEx.hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .addProcessors(propProcessor, aprilTagProcessor)
                .build();

        assert propProcessor != null;
        assert aprilTagProcessor != null;
        portal.setProcessorEnabled(propProcessor, true);
        portal.setProcessorEnabled(aprilTagProcessor, false);
    }

    public Prop.Location getPropLocation() {
        Prop.Location propLocation = propProcessor.getPropLocation();
        aprilTagProcessor.setPropLocation(propLocation);
        portal.setProcessorEnabled(aprilTagProcessor, true);
        portal.setProcessorEnabled(propProcessor, false);

//        propProcessor.close();

        return propLocation;
    }

    private void setupPropProcessor(LinearOpModeEx linearOpModeEx, Util.AllianceColor allianceColor, boolean startingClose) {
        if (allianceColor == Util.AllianceColor.RED) {
            if (startingClose) {
                propProcessor = new RightPropProcessor();
            } else {
                propProcessor = new LeftPropProcessor();
            }
            propProcessor.setDetectionColor(Prop.Color.RED);
        } else if (startingClose) {
            propProcessor = new LeftPropProcessor();
            propProcessor.setDetectionColor(Prop.Color.BLUE);
        } else {
            propProcessor = new RightPropProcessor();
            propProcessor.setDetectionColor(Prop.Color.BLUE);
        }
        propProcessor.setTelemetry(linearOpModeEx.telemetry);
    }

    private void setupAprilTagProcessor(LinearOpModeEx linearOpModeEx) {
        aprilTagProcessor = new cAprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
//                .setDrawAxes(true)
//                .setDrawTagID(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
                .setTagFamily(cAprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(Camera.fx, Camera.fy, Camera.cx, Camera.cy)
                .build();

        aprilTagProcessor.setDecimation(1);
        aprilTagProcessor.setTelemetry(linearOpModeEx.telemetry);
    }

    public void addToDashboard(FtcDashboard dashboard) {
        if (aprilTagProcessor != null) {
            dashboard.startCameraStream(aprilTagProcessor, 0);
        }
    }

    public void closePropProcessor() {
        portal.setProcessorEnabled(propProcessor, false);
        propProcessor.close();
    }

    public void closeAll() {
        portal.stopLiveView();
        portal.setProcessorEnabled(propProcessor, false);
        portal.setProcessorEnabled(aprilTagProcessor, false);
        portal.stopStreaming();
        portal.close();
    }

    public PixelLocation getPixelLocation() {
        return aprilTagProcessor.getPixelLocation();
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
//                    telemetryPacket.put("Null?", "No");
                    telemetryPacket.put("# April Tags Detected", detections.size());
                    backdropClear = (detections.size() == normalNumAprilTags);
                } else {
//                    telemetryPacket.put("Null?", "Yes");
                    backdropClear = false;
                }

                if (!backdropClear) {
                    timer.reset();
                }

//                telemetryPacket.put("Time", timer.seconds());
                telemetryPacket.put("Backdrop Clear", backdropClear);
                return timer.seconds() < buffer;
            }
        };
    }

    public Action relocalize(MecanumDrive drive) {
        return telemetryPacket -> {
            Pose2d pose = AprilTagPoseSolver.getRobotPoseBackdrop(
                    aprilTagProcessor::getFreshDetections,
                    drive::getPoseEstimate,
                    new Vector2d(1.5, 1.5)
            );

            // checks if there were no valid april tags seen
            if (!pose.position.equals(new Vector2d(0, 0))) {
                drive.setPoseEstimate(pose);
            }
            return false;
        };
    }

    public VisionPortal getPortal() {
        return this.portal;
    }
}
