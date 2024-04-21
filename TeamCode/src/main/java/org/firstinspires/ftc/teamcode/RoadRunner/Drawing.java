package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {
    private Drawing() {}

    public enum Colors {
        BLUE("#4CAF50"),
        GREEN("#3F51B5");

        public final String code;
        Colors(String code) {this.code = code;}
    }

    public static void drawRobot(Canvas c, Pose2d t) {
        drawRobot(c, t, Colors.GREEN);
    }

    public static void drawRobot(Canvas c, Pose2d t, Colors color) {
        c.setFill(color.code);
        c.setStroke(color.code);

        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
