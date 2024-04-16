package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.FarAutonomousOpMode;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

@Autonomous(name="Blue Far", group="Blue")
public class BlueFar extends LinearOpModeEx {

    FarAutonomousOpMode auto = null;
    FarAutonomousOpMode defaultAuto;
    Util.Path path = null;
    Util.Parking parking = null;
    boolean chosenCycles = false;
    int numCycles = 0;
    double timeout = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        defaultAuto = new FarAutonomousOpMode(this, Util.AllianceColor.BLUE);

        while (opModeInInit() && auto == null) {
            gamepadEx1.update();
            gamepadEx2.update();

            if (parking == null) {
                telemetry.clearAll();
                telemetry.addLine("Should autonomous mode park in the corner or in the middle?");
                telemetry.addLine("Press (Y/Δ) for corner, (B/O) for middle");

                if (gamepadEx1.wasJustPressed(Buttons.TRIANGLE) || gamepadEx2.wasJustPressed(Buttons.TRIANGLE)) {
                    parking = Util.Parking.CORNER;
                } else if (gamepadEx1.wasJustPressed(Buttons.CIRCLE) || gamepadEx2.wasJustPressed(Buttons.CIRCLE)) {
                    parking = Util.Parking.MIDDLE;
                }

            } else if (!chosenCycles) {
                telemetry.clearAll();
                telemetry.addLine("How many cycles should we attempt?");
                telemetry.addLine("Press DPAD UP to increase and DPAD Down to decrease");
                telemetry.addLine("Press (Y/Δ) to confirm");
                telemetry.addLine(String.format("Cycles: %s", numCycles));

                if (gamepadEx1.wasJustPressed(Buttons.DPAD_UP) || gamepadEx2.wasJustPressed(Buttons.DPAD_UP)) {
                    numCycles++;
                } else if (gamepadEx1.wasJustPressed(Buttons.DPAD_DOWN) || gamepadEx2.wasJustPressed(Buttons.DPAD_DOWN)) {
                    numCycles--;
                }

                if (numCycles < 0) numCycles = 0;
                if (numCycles > 1) numCycles = 1;

                if (gamepadEx1.wasJustPressed(Buttons.TRIANGLE) || gamepadEx2.wasJustPressed(Buttons.TRIANGLE)) {
                    chosenCycles = true;
                }

            } else if (numCycles > 0 && path == null) {
                telemetry.clearAll();
                telemetry.addLine("Would you like to go through stage door");
                telemetry.addLine("Press (Y/Δ) for yes, (B/O) for no");

                if (gamepadEx1.wasJustPressed(Buttons.TRIANGLE) || gamepadEx2.wasJustPressed(Buttons.TRIANGLE)) {
                    path = Util.Path.STAGE_DOOR;
                } else if (gamepadEx1.wasJustPressed(Buttons.CIRCLE) || gamepadEx2.wasJustPressed(Buttons.CIRCLE)) {
                    path = Util.Path.TRUSS;
                }

            } else if (getRuntime() > timeout) {
                break;
            } else {
                auto = new FarAutonomousOpMode(this, Util.AllianceColor.BLUE, parking, path, numCycles);
                break;
            }
            telemetry.update();
        }

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        if (auto == null) {
            auto = defaultAuto;
        }

        Prop.Location propLocation = auto.detectPropLocation();

        auto.runAll();
    }

}
