package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class BlueLocations {

    private final static double startingY = 62;
    private final static double backdropX = 52;
    private final static double stackX = -59.5;
    private final static double stageDoorStackY = 10.75;

    public enum Poses {
        CLOSE_STARTING(new Pose2d(15, startingY, Math.toRadians(90))),
        FAR_STARTING(new Pose2d(-38.5, startingY, Math.toRadians(90))),

        SPIKE_MARK_LEFT_CLOSE(new Pose2d(23, 40, Math.toRadians(90))),
        SPIKE_MARK_MIDDLE_CLOSE(new Pose2d(12, 33, Math.toRadians(90))),
        SPIKE_MARK_RIGHT_CLOSE(new Pose2d(5.5, 37, 0)),

        SPIKE_MARK_LEFT_FAR(new Pose2d(-27, 35, 0)),
        SPIKE_MARK_MIDDLE_FAR(new Pose2d(-34.5, 33, Math.toRadians(90))),
        SPIKE_MARK_RIGHT_FAR(new Pose2d(-46, 37, Math.toRadians(90))),

        BACKDROP_LEFT(new Pose2d(backdropX, 41.41, Math.toRadians(180))),
        BACKDROP_MIDDLE(new Pose2d(backdropX, 35.41, Math.toRadians(180))),
        BACKDROP_RIGHT(new Pose2d(backdropX, 29.41, Math.toRadians(180))),

        CYCLE_START_STAGE_DOOR(new Pose2d(28, stageDoorStackY, Math.toRadians(180))),
        CYCLE_START_STAGE_DOOR_2(new Pose2d(-20, stageDoorStackY, Math.toRadians(180))),
        STAGE_DOOR_STACK(new Pose2d(stackX, 11.125, Math.toRadians(180))),

        CYCLE_START_TRUSS(new Pose2d(20, 58, Math.toRadians(180))),
        CYCLE_START_TRUSS_2(new Pose2d(-30, 58, Math.toRadians(180))),
        TRUSS_STACK(new Pose2d(stackX, 35.125, Math.toRadians(180))),

        STAGE_DOOR_WAIT(new Pose2d(28, stageDoorStackY, Math.toRadians(225))),
        TRUSS_WAIT(new Pose2d(20, 58, Math.toRadians(135))),

        PARKING_MIDDLE(new Pose2d(54,  11, Math.toRadians(180))),
        PARKING_CORNER(new Pose2d(54, 59.5, Math.toRadians(180)));

        public final Pose2d pose;
        public final Vector2d vector;

        Poses(Pose2d pose) {
            this.pose = pose;
            this.vector = pose.position;
        }
    }

}
