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

public class RightPropProcessor extends PropProcessor implements CameraStreamSource {

	private final AtomicReference<Bitmap> lastFrame =
			new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

	private Mat testMat = new Mat();
	private Mat highMat = new Mat();
	private Mat lowMat = new Mat();
	private Mat finalMat = new Mat();
	private final double middleThreshold = 0.1;
	private final double rightThreshold = 0.1;
	Telemetry telemetry;

	Prop.Location propLocation;
	Prop.Color propColor = Prop.Color.RED;
	double rightPerc;
	double middlePerc;

	Rect RIGHT_RECTANGLE;
	Rect MIDDLE_RECTANGLE;

	@Override
	public void init(int width, int height, CameraCalibration calibration) {

		this.RIGHT_RECTANGLE = new Rect(
				new Point(11 * width / 16,height / 3.5),
				new Point(width, 2 * height / 3)
		);

		this.MIDDLE_RECTANGLE = new Rect(
				new Point(width / 10, height / 3.5),
				new Point(9 * width / 16, 2 * height / 3)
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
			Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
			Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

			Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
			Scalar highHSVRedUpper = new Scalar(180, 255, 255);

			Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
			Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);
			Core.bitwise_or(lowMat, highMat, finalMat);
		} else {
			Scalar blueHSVLower = new Scalar(85, 100, 20);
			Scalar blueHSVUpper = new Scalar(140, 255, 255);

			Core.inRange(testMat, blueHSVLower, blueHSVUpper, finalMat);
		}

		double leftBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];
		double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

		this.rightPerc = leftBox / RIGHT_RECTANGLE.area() / 255;
		this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]

		if(rightPerc > middlePerc
			&& rightPerc > rightThreshold) {
			propLocation = Prop.Location.RIGHT;
		} else if (middlePerc > rightPerc
					&& middlePerc > middleThreshold) {
			propLocation = Prop.Location.MIDDLE;
		} else {
			propLocation = Prop.Location.LEFT;
		}

		Scalar redBorder = new Scalar(255, 0, 0);
		Scalar greenBorder = new Scalar(0, 255, 0);

		switch (propLocation) {
			case LEFT:
				Imgproc.rectangle(frame, RIGHT_RECTANGLE, redBorder);
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
				break;
			case MIDDLE:
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);
				Imgproc.rectangle(frame, RIGHT_RECTANGLE, redBorder);
				break;
			case RIGHT:
				Imgproc.rectangle(frame, RIGHT_RECTANGLE, greenBorder);
				Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
				break;
		}

		Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
		Utils.matToBitmap(frame, b);
		lastFrame.set(b);

		if (telemetry != null) {
			updateTelemetry();
		}

		return null;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
	}

	public Prop.Location getPropLocation() {
		return this.propLocation;
	}

	public double[] getPercents() {
		return new double[]{rightPerc, middlePerc};
	}

	public void setTelemetry(Telemetry telemetry) {
		this.telemetry = telemetry;
	}

	public void updateTelemetry() {
		if (telemetry != null) {
			telemetry.addLine("Prop Processor")
					.addData("Right Percent", rightPerc)
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