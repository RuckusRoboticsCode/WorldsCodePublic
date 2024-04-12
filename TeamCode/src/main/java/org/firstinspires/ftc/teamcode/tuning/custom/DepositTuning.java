package org.firstinspires.ftc.teamcode.tuning.custom;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Helper.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.OpModes.OpModeEx;
import org.firstinspires.ftc.teamcode.Tele.Subsystems.HardwareNames;

@TeleOp
@Config
@Disabled
public class DepositTuning extends OpModeEx {

    private ServoImplEx depositLeft = null;
    private ServoImplEx depositRight = null;
    public enum ServoControl {
        LEFT,
        RIGHT,
        BOTH;
    }
    public static ServoControl pos = ServoControl.LEFT;
    public static double leftPos = 0.5;
    public static double rightPos = 0.5;

    @Override
    public void init() {
        super.init();
        depositLeft = hardwareMap.get(ServoImplEx.class, HardwareNames.DEPOSIT_SERVO_LEFT);
        depositRight = hardwareMap.get(ServoImplEx.class, HardwareNames.DEPOSIT_SERVO_RIGHT);
    }

    @Override
    public void loop() {
        super.loop();
        if (pos == ServoControl.LEFT) {
            if (!depositLeft.isPwmEnabled()) {
                depositLeft.setPwmEnable();
            }
            if (depositRight.isPwmEnabled()) {
                depositRight.setPwmDisable();
            }
            depositLeft.setPosition(leftPos);
        } else if (pos == ServoControl.RIGHT){
            if (depositLeft.isPwmEnabled()) {
                depositLeft.setPwmDisable();
            }
            if (!depositRight.isPwmEnabled()) {
                depositRight.setPwmEnable();
            }
            depositRight.setPosition(rightPos);
        } else {
            depositLeft.setPwmEnable();
            depositRight.setPwmEnable();

            depositLeft.setPosition(leftPos);
            depositRight.setPosition(rightPos);
        }
    }
}
