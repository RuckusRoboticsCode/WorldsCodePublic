package org.firstinspires.ftc.teamcode.OpModes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.Tele.GamepadEx.GamepadEx;

import java.util.List;

@Photon
public class LinearOpModeEx extends LinearOpMode {

    public GamepadEx gamepadEx1 = null;
    public GamepadEx gamepadEx2 = null;
    public LynxModule controlHub = null;

    @Override
    public void runOpMode() throws InterruptedException {
        if (gamepad1 != null) {
            gamepadEx1 = new GamepadEx(gamepad1);
        }
        if (gamepad2 != null) {
            gamepadEx2 = new GamepadEx(gamepad2);
        }

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        if(allHubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(allHubs.get(0).getSerialNumber())) {
            controlHub = allHubs.get(0);
            allHubs.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } else {
            controlHub = allHubs.get(1);
            allHubs.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
}
