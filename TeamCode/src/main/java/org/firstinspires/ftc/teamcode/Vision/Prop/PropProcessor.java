package org.firstinspires.ftc.teamcode.Vision.Prop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class PropProcessor implements VisionProcessor {

    public abstract void setDetectionColor(Prop.Color color);
    public abstract void setTelemetry(Telemetry telemetry);

    public abstract Prop.Location getPropLocation();

    public abstract void close();

}
