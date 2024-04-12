package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.LinearSlides;

@TeleOp(name="TeleOp")
public class Tele extends OpModeEx {

    private Deposit deposit;
    private Drivetrain drivetrain;
    private DroneLauncher droneLauncher;
    private Intake intake;
    private LinearSlides linearSlides;
    private IntermittentVoltageSensor intermittentVoltageSensor;

    @Override
    public void init() {
        super.init();
        makeHardware();
        initHardware();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        super.loop();
        read();
        update();
    }

    private void makeHardware() {
        deposit = new Deposit();
//        deposit.setGamepadEx(gamepadEx2);
        deposit.setGamepadExs(gamepadEx1, gamepadEx2);

        drivetrain = new Drivetrain();
        drivetrain.setGamepadEx(gamepadEx1);

        droneLauncher = new DroneLauncher();
        droneLauncher.setGamepadEx(gamepadEx2);

        intake = new Intake();
        intake.setGamepadEx(gamepadEx1);

        linearSlides = new LinearSlides();
        linearSlides.setGamepadEx(gamepadEx2);
        linearSlides.setTelemetry(telemetry);

        intermittentVoltageSensor = new IntermittentVoltageSensor(hardwareMap, 1500);
    }

    private void initHardware() {
        deposit.init(hardwareMap);
        drivetrain.init(hardwareMap);
        droneLauncher.init(hardwareMap);
        intake.init(hardwareMap);
        linearSlides.init(hardwareMap);
    }

    private void read() {
        double voltage = intermittentVoltageSensor.getVoltage();
//        double voltage = 12.0;
        linearSlides.setVoltage(voltage);
        intake.setVoltage(voltage);
        deposit.read();
        intake.read(deposit);
        linearSlides.read();
    }

    private void update() {
        deposit.update();
        drivetrain.update();
        droneLauncher.update();
        intake.update();
        linearSlides.update();
        linearSlides.updateTelemetry();
    }

}