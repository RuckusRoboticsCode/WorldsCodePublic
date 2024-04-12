package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.CloseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.Auto.FarAutonomousOpMode;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

@Config
@Autonomous
public class FarAutoTesting extends LinearOpModeEx {

    FarAutonomousOpMode auto;
    public static StopState stopState = StopState.PRELOAD;
    public static Util.AllianceColor allianceColor = Util.AllianceColor.BLUE;
    public static Util.Parking parking = Util.Parking.MIDDLE;
    public static Util.Path path = Util.Path.STAGE_DOOR;

    public enum StopState {
        PRELOAD,
        PRELOAD_PARK,
        FIRST_CYCLE,
        FIRST_CYCLE_PARK,
        SECOND_CYCLE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        auto = new FarAutonomousOpMode(
                this,
                allianceColor,
                parking,
                path,
                1
        );

        waitForStart();

        Prop.Location propLocation = auto.detectPropLocation();

        Actions.runBlocking(auto.getPreLoad());

        if (stopState == StopState.PRELOAD) {
            requestOpModeStop();
        }

        if (stopState == StopState.PRELOAD_PARK) {
            Actions.runBlocking(auto.getPark());
            requestOpModeStop();
        }

        Actions.runBlocking(auto.getFirstCycle());

        if (stopState == StopState.FIRST_CYCLE) {
            requestOpModeStop();
        }

        if (stopState == StopState.FIRST_CYCLE_PARK) {
            Actions.runBlocking(auto.getPark());
            requestOpModeStop();
        }
    }
}
