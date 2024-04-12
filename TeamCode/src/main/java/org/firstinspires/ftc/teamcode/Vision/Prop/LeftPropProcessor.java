package org.firstinspires.ftc.teamcode.Vision.Prop;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


import java.util.concurrent.atomic.AtomicReference;

public class LeftPropProcessor extends PropProcessor implements CameraStreamSource {

	private final AtomicReference<Bitmap> lastFrame =
			new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

	private final Mat testMat = new Mat();
	private final Mat highMat = new Mat();
	private final Mat lowMat = new Mat();
	private final Mat finalMat = new Mat();

	private final double middleThreshold = 0.15;
	private final double leftThreshold = 0.2;
	private Telemetry telemetry;

	private Prop.Location propLocation;
	private Prop.Color propColor;
	private double leftPerc;
	private double middlePerc;

	private Rect LEFT_RECTANGLE;
	private Rect MIDDLE_RECTANGLE;

	@Override
	public void init(int width, int height, CameraCalibration calibration) {

		this.LEFT_RECTANGLE = new Rect(
				new Point(0, 0.286 * height),
				new Point(0.222 * width, 0.666 * height)
		);

		this.MIDDLE_RECTANGLE = new Rect(
				new Point(0.3125 * width, 0.286 * height),
				new Point(0.9 * width, 0.666 * height)
		);

		lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
	}

	public void setDetectionColor(Prop.Color propColor) {
		this.propColor = propColor;
	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

		if(propColor == Prop.Color.RED) {

			Scalar lowHSVRedLower = new Scalar(0, 100, 20);
			Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

			Scalar redHSVRedLower = new Scalar(160, 100, 20);
			Scalar highHSVRedUpper = new Scalar(179, 255, 255);

			Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
			Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

			Core.bitwise_or(lowMat, highMat, finalMat);
		} else {
			Scalar blueHSVLower = new Scalar(85, 100, 20);
			Scalar blueHSVUpper = new Scalar(140, 255, 255);

			Core.inRange(testMat, blueHSVLower, blueHSVUpper, finalMat);
		}

		double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
		double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

		this.leftPerc = leftBox / LEFT_RECTANGLE.area() / 255;
		this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255;

		if(leftPerc > middlePerc
			&& leftPerc > leftThreshold) {
			propLocation = Prop.Location.LEFT;
		} else if (middlePerc > leftPerc
					&& middlePerc > middleThreshold) {
			propLocation = Prop.Location.MIDDLE;
		} else {
			propLocation = Prop.Location.RIGHT;
		}

		Scalar redBorder = new Scalar(255, 0, 0);
		Scalar greenBorder = new Scalar(0, 255, 0);

		switch (propLocation) {
			case LEFT:
				Imgproc.rectangle(frame, LEFT_RECTANGLE, greenBorder);
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
				break;
			case MIDDLE:
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);
				Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
				break;
			case RIGHT:
				Imgproc.rectangle(frame, LEFT_RECTANGLE, redBorder);
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
				break;
		}

		Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
		Utils.matToBitmap(frame, b);
		lastFrame.set(b);

		return null;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
	}

	public Prop.Location getPropLocation() {
		return this.propLocation;
	}

	public double[] getPercents() {
		return new double[]{leftPerc, middlePerc};
	}

	public void setTelemetry(Telemetry telemetry) {
		this.telemetry = telemetry;
	}

	public void updateTelemetry() {
		if (telemetry != null) {
			telemetry.addLine("Prop Processor")
					.addData("Left Percent", leftPerc)
					.addData("Middle Percent", middlePerc)
					.addData("Prop Location", propLocation);
		}
	}

	@Override
	public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
		continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
	}

	public void close() {
		testMat.release();
		highMat.release();
		lowMat.release();
		finalMat.release();
	}
}