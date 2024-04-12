package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;

@Disabled
@TeleOp
public class VoltageTest extends OpModeEx {

    IntermittentVoltageSensor intermittentVoltageSensor;

    @Override
    public void init() {
        super.init();
        intermittentVoltageSensor = new IntermittentVoltageSensor(
                hardwareMap,
                15
        );
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Intermittent Read", intermittentVoltageSensor.getVoltage());
        telemetry.addData("Cached Read", intermittentVoltageSensor.getCachedVoltage());
        telemetry.addData("Fresh Read", intermittentVoltageSensor.getFreshVoltage());
    }
}
