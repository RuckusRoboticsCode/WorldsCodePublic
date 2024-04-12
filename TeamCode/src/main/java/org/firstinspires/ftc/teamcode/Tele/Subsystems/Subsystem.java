package org.firstinspires.ftc.teamcode.Tele.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;

public interface Subsystem {

    default void read() {}
    void setGamepadEx(GamepadEx gamepadEx);
    void setTelemetry(Telemetry telemetry);
    default void updateTelemetry() {}
    void init(HardwareMap hardwareMap);
    void update();
}
