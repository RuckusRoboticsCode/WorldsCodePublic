package org.firstinspires.ftc.teamcode.Auto.Red;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.CloseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.Auto.Util;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

@Autonomous(name="Red Close", group="Red")
public class RedClose extends LinearOpModeEx {

    CloseAutonomousOpMode auto = null;
    CloseAutonomousOpMode defaultAuto;
    Util.Path path = null;
    Util.Parking parking = null;
    boolean chosenCycles = false;
    int numCycles = 0;
    double timeout = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        defaultAuto = new CloseAutonomousOpMode(this, Util.AllianceColor.RED);

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
                if (numCycles > 2) numCycles = 2;

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
                telemetry.clearAll();
                telemetry.update();
                break;
            } else {
                auto = new CloseAutonomousOpMode(this, Util.AllianceColor.RED, parking, path, numCycles);

                telemetry.clearAll();
                telemetry.addData("Parking",
                        parking == Util.Parking.MIDDLE ? "Middle" : "Corner"
                );
                telemetry.addData("Num Cycles", numCycles);
                if (numCycles == 1) {
                    telemetry.addData("Path",
                            path == Util.Path.STAGE_DOOR ? "Stagedoor" : "Truss"
                    );
                }

                break;
            }
            telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addData("Parking",
                parking == Util.Parking.MIDDLE ? "Middle" : "Corner"
        );
        telemetry.addData("Num Cycles", numCycles);
        if (numCycles == 1) {
            telemetry.addData("Path",
                    path == Util.Path.STAGE_DOOR ? "Stagedoor" : "Truss"
            );
        }
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        if (auto == null) {
            auto = defaultAuto;
        }

        Prop.Location propLocation = auto.detectPropLocation();

        Actions.runBlocking(auto.getPreLoad());
        // TODO: change RIGHT to LEFT for red side
        if (numCycles == 0 || (path == Util.Path.TRUSS && propLocation == Prop.Location.LEFT)) {
            Actions.runBlocking(auto.getPark());
        } else if (numCycles == 1){
            Actions.runBlocking(auto.getFirstCycle());
            if (getRuntime() < 25) {
                Actions.runBlocking(auto.getPark());
            }
        } else if (numCycles == 2) {
            Actions.runBlocking(auto.getSecondCycle());
            if (getRuntime() < 25) {
                Actions.runBlocking(auto.getPark());
            }
        }
    }

}
