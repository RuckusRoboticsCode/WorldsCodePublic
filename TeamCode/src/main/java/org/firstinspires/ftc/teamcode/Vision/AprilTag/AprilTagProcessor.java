package org.firstinspires.ftc.teamcode.Vision.AprilTag;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class AprilTagProcessor extends cAprilTagProcessorImpl implements CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Prop.Location propLocation = Prop.Location.MIDDLE;

    public static int boxHeight = 75;
    public static int boxWidth = 100;
    public static int boxVerticalOffset = 50;
    private Rect leftROI = new Rect();
    private Rect rightROI = new Rect();

    private final Mat hsvMat = new Mat();
    private final Mat yellowMat = new Mat();
    private final Scalar yellowLower = new Scalar(25, 100, 100);
    private final Scalar yellowUpper = new Scalar(35, 255, 255);
    private final Scalar redBorder = new Scalar(255, 0, 0);
    private final Scalar greenBorder = new Scalar(0, 255, 0);
    private Mat leftYellow = new Mat();
    private Mat rightYellow = new Mat();

    public static double threshold = 0.05;
    private PixelLocation pixelLocation = PixelLocation.NOT_FOUND;

    public AprilTagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads, boolean suppressCalibrationWarnings) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline, drawTagID, tagFamily, threads, suppressCalibrationWarnings);
    }

    public void setPropLocation(Prop.Location propLocation) {
        this.propLocation = propLocation;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        super.init(width, height, calibration);
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        Bitmap b = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, b);
        lastFrame.set(b);

        super.processFrame(input, captureTimeNanos);

        ArrayList<AprilTagDetection> aprilTagDetections = getDetections();
        Point center = new Point();

        switch (propLocation) {
            case LEFT:
                for (AprilTagDetection detection : aprilTagDetections) {
                    if (detection.id == 1 || detection.id == 4) {
                        center = detection.center;
                        makeRectangles(center);
                        break;
                    }
                }
                break;
            case MIDDLE:
                for (AprilTagDetection detection : aprilTagDetections) {
                    if (detection.id == 2 || detection.id == 5) {
                        center = detection.center;
                        makeRectangles(center);
                        break;
                    }
                }
                break;
            case RIGHT:
                for (AprilTagDetection detection : aprilTagDetections) {
                    if (detection.id == 3 || detection.id == 6) {
                        center = detection.center;
                        makeRectangles(center);
                        break;
                    }
                }
                break;
        }

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMat);
        leftYellow = yellowMat.submat(leftROI);
        rightYellow = yellowMat.submat(rightROI);

        double leftPerc = Core.sumElems(leftYellow).val[0] / leftROI.area() / 255;
        double rightPerc = Core.sumElems(rightYellow).val[0] / rightROI.area() / 255;

        if (leftPerc > threshold && leftPerc >= rightPerc) {
            pixelLocation = PixelLocation.LEFT;
            Imgproc.rectangle(input, leftROI, greenBorder);
            Imgproc.rectangle(input, rightROI, redBorder);
        } else if (rightPerc > threshold && rightPerc >= leftPerc) {
            pixelLocation = PixelLocation.RIGHT;
            Imgproc.rectangle(input, leftROI, redBorder);
            Imgproc.rectangle(input, rightROI, greenBorder);
        } else {
            pixelLocation = PixelLocation.NOT_FOUND;
            Imgproc.rectangle(input, leftROI, redBorder);
            Imgproc.rectangle(input, rightROI, redBorder);
        }
        Imgproc.drawMarker(input, center, greenBorder, Imgproc.MARKER_CROSS, 5);

        return null;
    }

    private void makeRectangles(Point center) {

        double lowerY = Math.min(Math.max(center.y - boxVerticalOffset, 0), MAX_HEIGHT);

        leftROI = new Rect(
                new Point(center.x, lowerY),
                new Point(Math.max(center.x - boxWidth, 0), Math.max(lowerY - boxHeight, 0))
        );

        rightROI = new Rect(
                new Point(center.x, lowerY),
                new Point(Math.min(center.x + boxWidth, MAX_WIDTH), Math.max(lowerY - boxHeight, 0))
        );
    }

    public PixelLocation getPixelLocation() {
        return pixelLocation;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
