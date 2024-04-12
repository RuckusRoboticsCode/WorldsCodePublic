package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;
import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;

public class DroneLauncher implements Subsystem {

    private GamepadEx gamepadEx = null;
    private Servo droneServo = null;
    private Telemetry telemetry = null;
    private LaunchPosition position = LaunchPosition.LOAD;

    enum LaunchPosition {
        LOAD(0.49),
        LAUNCH(0.85);

        public final double pos;

        LaunchPosition(double pos) {
            this.pos = pos;
        }
    }

    @Override
    public void setGamepadEx(GamepadEx gamepadEx) {
        this.gamepadEx = gamepadEx;
    }

    @Override
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        droneServo = new CachingServo(hardwareMap.get(Servo.class, HardwareNames.DRONE_LAUNCHER), 0.01);
        position = LaunchPosition.LOAD;

        droneServo.setPosition(position.pos);
    }

    @Override
    public void update() {
        if (gamepadEx != null) {
            if (gamepadEx.wasJustPressed(Buttons.DPAD_UP) ||
                    gamepadEx.wasJustPressed(Buttons.DPAD_DOWN) ||
                    gamepadEx.wasJustPressed(Buttons.DPAD_LEFT) ||
                    gamepadEx.wasJustPressed(Buttons.DPAD_RIGHT)) {
                if (position == LaunchPosition.LOAD) {
                    position = LaunchPosition.LAUNCH;
                } else {
                    position = LaunchPosition.LOAD;
                }
            }
        }
        droneServo.setPosition(position.pos);
    }
}
