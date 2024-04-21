package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Tele.GamepadEx.Buttons;

@Disabled
@Autonomous
public class DropOneAutoTesting extends LinearOpModeEx {

    Deposit deposit;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        deposit = new Deposit(this);
        Actions.runBlocking(deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE));

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.update();
            if (gamepadEx1.wasJustPressed(Buttons.LEFT_BUMPER)) {
                Actions.runBlocking(deposit.dropOnePixel());
            }
        }
    }
}
