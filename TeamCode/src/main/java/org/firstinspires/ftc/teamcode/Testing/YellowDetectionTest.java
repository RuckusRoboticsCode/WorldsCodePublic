package org.firstinspires.ftc.teamcode.Testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.cAprilTagProcessor;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
//@Config
@Autonomous(name="Yellow Detection", group="Test")
public class YellowDetectionTest extends LinearOpModeEx {

    public static Prop.Location propLocation = Prop.Location.MIDDLE;
    AprilTagProcessor processor;
    VisionPortal portal;

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

        processor.setPropLocation(propLocation);
        processor.setDecimation(3);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .addProcessor(processor)
                .build();

        portal.setProcessorEnabled(processor, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(portal, 0);

        waitForStart();

        while (opModeIsActive()) {
            processor.setPropLocation(propLocation);

            telemetry.addData("Yellow Location", processor.getPixelLocation().toString());

            telemetry.update();
        }
    }
}
