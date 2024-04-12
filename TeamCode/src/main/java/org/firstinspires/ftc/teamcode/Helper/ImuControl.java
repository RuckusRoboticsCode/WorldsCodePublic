package org.firstinspires.ftc.teamcode.Helper;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuControl {

    private final double startingHeadingRad;
    private final IMU imu;
    private final boolean positiveOnly;

    public ImuControl(LazyImu lazyImu, double startingHeadingRad) {
        this(lazyImu, startingHeadingRad, true);
    }

    public ImuControl(IMU imu, double startingHeadingRad) {
        this(imu, startingHeadingRad, true);
    }

    public ImuControl(LazyImu imu, double startingHeadingRad, boolean positiveAngleOnly) {
        this.startingHeadingRad = startingHeadingRad;
        this.imu = imu.get();
        this.positiveOnly = positiveAngleOnly;
        this.imu.resetYaw();
    }

    public ImuControl(IMU imu, double startingHeadingRad, boolean positiveAngleOnly) {
        this.startingHeadingRad = startingHeadingRad;
        this.imu = imu;
        this.positiveOnly = positiveAngleOnly;
        this.imu.resetYaw();
    }

    public double getHeadingRad() {
        return angleWrapRad(startingHeadingRad + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    private double angleWrapRad(double headingRad) {
        double modifiedAngle = headingRad % (2 * Math.PI);
        modifiedAngle += (2 * Math.PI);
        modifiedAngle %= (2 * Math.PI);

        if (!positiveOnly) {
            if (modifiedAngle > Math.PI) {
                modifiedAngle -= (2 * Math.PI);
            }
        }
        return modifiedAngle;
    }
}
