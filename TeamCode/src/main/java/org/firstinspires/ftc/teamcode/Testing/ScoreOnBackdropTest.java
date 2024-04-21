package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.PixelLocation;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

@Disabled
@Config
@Autonomous
public class ScoreOnBackdropTest extends LinearOpModeEx {

    private MecanumDrive mecanumDrive;
    private Vision vision;
    private Pose2d startingPose = new Pose2d(34, 35.41, Math.toRadians(180));
    public static PixelLocation pixelLocation = PixelLocation.RIGHT;
    public static Util.AllianceColor allianceColor = Util.AllianceColor.BLUE;
    public static Prop.Location propLocation = Prop.Location.MIDDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive = new MecanumDrive(
                hardwareMap, startingPose
        );
        vision = new Vision(
                this, Util.AllianceColor.BLUE,
                true
        );

        vision.getPropLocation();

        waitForStart();

        if (allianceColor == Util.AllianceColor.RED) {
            mecanumDrive.setPoseEstimate(
                    new Pose2d(
                            startingPose.position.x,
                            -startingPose.position.y,
                            startingPose.heading.inverse().toDouble()
                    )
            );
        }

        Actions.runBlocking(
            mecanumDrive.scoreBackdrop(
                    allianceColor,
                    vision::getPixelLocation,
                    this::getPropLocation
            )
        );

        while (opModeIsActive()) {

        }
    }

    public PixelLocation getPixelLocation() {
        return pixelLocation;
    }

    public Prop.Location getPropLocation() {
        return propLocation;
    }
}
