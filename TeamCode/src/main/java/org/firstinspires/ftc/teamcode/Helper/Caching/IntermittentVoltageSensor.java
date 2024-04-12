package org.firstinspires.ftc.teamcode.Helper.Caching;

import com.outoftheboxrobotics.photoncore.PeriodicSupplier;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntermittentVoltageSensor {

    private final PhotonLynxVoltageSensor voltageSensor;
//    private final PeriodicSupplier<Double> voltage;
    private double voltage = 12.0;
    private final double periodMs;
    private final ElapsedTime timer = new ElapsedTime();

    public IntermittentVoltageSensor(HardwareMap hardwareMap, double readTimeMs) {
        voltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
//        voltage = new PeriodicSupplier<>(voltageSensor::getVoltageAsync, (long) readTimeMs);
//        voltage.update();
        this.periodMs = readTimeMs;
        timer.reset();
    }

    public double getVoltage() {
//        voltage.update();
//        return voltage.get();
        if (timer.milliseconds() > periodMs) {
            voltage = voltageSensor.getVoltage();
            timer.reset();
        }
        return voltage;
    }

    public double getFreshVoltage() {
        return voltageSensor.getVoltage();
    }

    public double getCachedVoltage() {
        return voltageSensor.getCachedVoltage();
    }

    public VoltageSensor getVoltageSensor() {
        return (VoltageSensor) voltageSensor;
    }
}
