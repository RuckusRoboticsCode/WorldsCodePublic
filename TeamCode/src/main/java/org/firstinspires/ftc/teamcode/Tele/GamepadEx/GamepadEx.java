package org.firstinspires.ftc.teamcode.Tele.GamepadEx;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Helper.BooleanEdgeDetector;

import java.util.ArrayList;

public class GamepadEx {

    private final Gamepad gamepad;
    private final Gamepad currentState = new Gamepad();
    private double deadzone;

    private final BooleanEdgeDetector CIRCLE_DETECTOR;
    private final BooleanEdgeDetector TRIANGLE_DETECTOR;
    private final BooleanEdgeDetector SQUARE_DETECTOR;
    private final BooleanEdgeDetector CROSS_DETECTOR;

    private final BooleanEdgeDetector DPAD_RIGHT_DETECTOR;
    private final BooleanEdgeDetector DPAD_UP_DETECTOR;
    private final BooleanEdgeDetector DPAD_LEFT_DETECTOR;
    private final BooleanEdgeDetector DPAD_DOWN_DETECTOR;

    private final BooleanEdgeDetector LEFT_BUMPER_DETECTOR;
    private final BooleanEdgeDetector RIGHT_BUMPER_DETECTOR;

    private final BooleanEdgeDetector OPTION_DETECTOR;
    private final BooleanEdgeDetector TOUCHPAD_DETECTOR;
    private final BooleanEdgeDetector SHARE_DETECTOR;

    private final ArrayList<BooleanEdgeDetector> booleanEdgeDetectors = new ArrayList<>();

    public GamepadEx(Gamepad gamepad) {
        this(gamepad, 0.05);
    }

    public GamepadEx(Gamepad gamepad, double deadzone) {
        this.deadzone = deadzone;
        this.gamepad = gamepad;
        currentState.copy(gamepad);

        CIRCLE_DETECTOR = new BooleanEdgeDetector(() -> currentState.circle);
        TRIANGLE_DETECTOR = new BooleanEdgeDetector(() -> currentState.triangle);
        SQUARE_DETECTOR = new BooleanEdgeDetector(() -> currentState.square);
        CROSS_DETECTOR = new BooleanEdgeDetector(() -> currentState.cross);

        DPAD_RIGHT_DETECTOR = new BooleanEdgeDetector(() -> currentState.dpad_right);
        DPAD_UP_DETECTOR = new BooleanEdgeDetector(() -> currentState.dpad_up);
        DPAD_LEFT_DETECTOR = new BooleanEdgeDetector(() -> currentState.dpad_left);
        DPAD_DOWN_DETECTOR = new BooleanEdgeDetector(() -> currentState.dpad_down);

        LEFT_BUMPER_DETECTOR = new BooleanEdgeDetector(() -> currentState.left_bumper);
        RIGHT_BUMPER_DETECTOR = new BooleanEdgeDetector(() -> currentState.right_bumper);

        OPTION_DETECTOR = new BooleanEdgeDetector(() -> currentState.options);
        TOUCHPAD_DETECTOR = new BooleanEdgeDetector(() -> currentState.touchpad);
        SHARE_DETECTOR = new BooleanEdgeDetector(() -> currentState.share);

        booleanEdgeDetectors.add(CIRCLE_DETECTOR);
        booleanEdgeDetectors.add(TRIANGLE_DETECTOR);
        booleanEdgeDetectors.add(SQUARE_DETECTOR);
        booleanEdgeDetectors.add(CROSS_DETECTOR);

        booleanEdgeDetectors.add(DPAD_RIGHT_DETECTOR);
        booleanEdgeDetectors.add(DPAD_UP_DETECTOR);
        booleanEdgeDetectors.add(DPAD_LEFT_DETECTOR);
        booleanEdgeDetectors.add(DPAD_DOWN_DETECTOR);

        booleanEdgeDetectors.add(LEFT_BUMPER_DETECTOR);
        booleanEdgeDetectors.add(RIGHT_BUMPER_DETECTOR);

        booleanEdgeDetectors.add(OPTION_DETECTOR);
        booleanEdgeDetectors.add(TOUCHPAD_DETECTOR);
        booleanEdgeDetectors.add(SHARE_DETECTOR);
    }

    public void update() {
        currentState.copy(gamepad);
        for (BooleanEdgeDetector detector : booleanEdgeDetectors) {
            detector.update();
        }
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    private BooleanEdgeDetector getEdgeDetector(Buttons button) {
        switch (button) {
            case CIRCLE:
                return CIRCLE_DETECTOR;
            case TRIANGLE:
                return TRIANGLE_DETECTOR;
            case SQUARE:
                return SQUARE_DETECTOR;
            case CROSS:
                return CROSS_DETECTOR;
            case DPAD_RIGHT:
                return DPAD_RIGHT_DETECTOR;
            case DPAD_UP:
                return DPAD_UP_DETECTOR;
            case DPAD_LEFT:
                return DPAD_LEFT_DETECTOR;
            case DPAD_DOWN:
                return DPAD_DOWN_DETECTOR;
            case LEFT_BUMPER:
                return LEFT_BUMPER_DETECTOR;
            case RIGHT_BUMPER:
                return RIGHT_BUMPER_DETECTOR;
            case OPTION:
                return OPTION_DETECTOR;
            case TOUCHPAD:
                return TOUCHPAD_DETECTOR;
            case SHARE:
                return SHARE_DETECTOR;
        }
        return null;
    }

    public boolean getButton(Buttons button) {
        BooleanEdgeDetector detector = getEdgeDetector(button);
        if (detector != null) {
            return detector.getCurrentValue();
        }
        return false;
    }

    private boolean getButtonPrevious(Buttons button) {
        BooleanEdgeDetector detector = getEdgeDetector(button);
        if (detector != null) {
            return detector.getPreviousValue();
        }
        return false;
    }

    public boolean wasJustPressed(Buttons button) {
        BooleanEdgeDetector detector = getEdgeDetector(button);
        if (detector != null) {
            return detector.wasJustTriggered();
        }
        return false;
    }

    public boolean wasJustReleased(Buttons button) {
        BooleanEdgeDetector detector = getEdgeDetector(button);
        if (detector != null) {
            return detector.wasJustReleased();
        }
        return false;
    }

    public boolean stateJustChanged(Buttons button) {
        BooleanEdgeDetector detector = getEdgeDetector(button);
        if (detector != null) {
            return (detector.getPreviousValue() != detector.getCurrentValue());
        }
        return false;
    }

    public double getLeftTrigger() {
        if (Math.abs(currentState.left_trigger) > deadzone) {
            return currentState.left_trigger;
        }
        return 0;
    }

    public double getRightTrigger() {
        if (Math.abs(currentState.right_trigger) > deadzone) {
            return currentState.right_trigger;
        }
        return 0;
    }

    public double getLeftY() {
        if (Math.abs(-currentState.left_stick_y) > deadzone) {
            return -currentState.left_stick_y;
        }
        return 0;
    }

    public double getLeftX() {
        if (Math.abs(currentState.left_stick_x) > deadzone) {
            return currentState.left_stick_x;
        }
        return 0;
    }

    public double getRightY() {
        if (Math.abs(currentState.right_stick_y) > deadzone) {
            return -currentState.right_stick_y;
        }
        return 0;
    }

    public double getRightX() {
        if (Math.abs(currentState.right_stick_x) > deadzone) {
            return currentState.right_stick_x;
        }
        return 0;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }
}
