package org.firstinspires.ftc.teamcode.Helper;

import java.util.function.BooleanSupplier;

public class BooleanEdgeDetector {

    private BooleanSupplier booleanSupplier = null;
    private boolean previousValue = false;
    private boolean currentValue = false;

    public BooleanEdgeDetector(BooleanSupplier booleanSupplier) {
        this.booleanSupplier = booleanSupplier;
    }

    public void update() {
        previousValue = currentValue;
        currentValue = booleanSupplier.getAsBoolean();
    }

    public boolean wasJustTriggered() {
        return (currentValue && !previousValue);
    }

    public boolean wasJustReleased() {
        return (!currentValue && previousValue);
    }

    public boolean getCurrentValue() {
        return currentValue;
    }

    public boolean getPreviousValue() {
        return previousValue;
    }
}
