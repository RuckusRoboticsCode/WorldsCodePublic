package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryAction;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Pose2d closeStarting = new Pose2d(15, 62, Math.toRadians(90));
        Pose2d farStarting = new Pose2d(-38.5, 62, Math.toRadians(90));
        double servoWaitTime = 0.6;

        Constraints constraints = new Constraints(70, 70, Math.toRadians(240), Math.toRadians(180), 16.25);
        RoadRunnerBotEntity closeBot = new RoadRunnerBotEntity(
                meepMeep,
                constraints,
                16.5,
                16.5,
                closeStarting,
                new ColorSchemeBlueLight(),
                1,
                DriveTrainType.MECANUM,
                false
        );

        RoadRunnerBotEntity farBot = new RoadRunnerBotEntity(
                meepMeep,
                constraints,
                16.5,
                16.5,
                farStarting,
                new ColorSchemeBlueLight(),
                1,
                DriveTrainType.MECANUM,
                false
        );

        Action farLeftTruss = farBot.getDrive().actionBuilder(farStarting)
                .strafeToSplineHeading(new Vector2d(-38, 43), Math.toRadians(115))
                .splineToLinearHeading(new Pose2d(-27, 35, Math.toRadians(135)), Math.toRadians(315))

                .strafeTo(new Vector2d(-41, 38))
                .splineToSplineHeading(new Pose2d(-59.5, 35.25, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-51.5, 35.25))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-59.5, 35.25))

                .waitSeconds(2)
                .strafeTo(new Vector2d(-59.5+1e-6, 35.25))
                .splineToSplineHeading(new Pose2d(-28, 58, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(20, 58))

                .waitSeconds(3)
                .strafeTo(new Vector2d(20+1e-6, 58))
                .splineToSplineHeading(new Pose2d(51, 41, Math.toRadians(180)), Math.toRadians(0))

                .waitSeconds(1)

                .splineToSplineHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-30, 58))
                .splineToSplineHeading(new Pose2d(-59.5, 35.25, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(2)

                .strafeTo(new Vector2d(-59.5+1e-6, 35.25))
                .splineToSplineHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(20, 58))
                .splineToSplineHeading(new Pose2d(51, 37, Math.toRadians(180)), Math.toRadians(0))

                .build();

        Action farLeftStageDoor = farBot.getDrive().actionBuilder(farStarting)
                .strafeToSplineHeading(new Vector2d(-38, 43), Math.toRadians(115))
                .splineToLinearHeading(new Pose2d(-27, 35, Math.toRadians(135)), Math.toRadians(315))

                .strafeTo(new Vector2d(-39, 35))
                .splineToSplineHeading(new Pose2d(-59.5, 11, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-51.5, 11))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-59.5, 11))

                .waitSeconds(2)

                .strafeToConstantHeading(new Vector2d(0, 11))
                .strafeToSplineHeading(new Vector2d(35, 11), Math.toRadians(225))
                .waitSeconds(5)
                .setTangent(Math.toRadians(45))

//                .splineToSplineHeading(new Pose2d(45, 41, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(45, 41, Math.toRadians(180)), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(45, 41), Math.toRadians(180))
                .waitSeconds(1)

                .splineToConstantHeading(new Vector2d(25, 11), Math.toRadians(180))

                .strafeTo(new Vector2d(-59.5, 11))
                .waitSeconds(2)

//                .strafeTo(new Vector2d(25, 11))
                .strafeToConstantHeading(new Vector2d(0, 11))
                .strafeToSplineHeading(new Vector2d(25, 11), Math.toRadians(225))
                .waitSeconds(5)
                .setTangent(Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(51, 29.5, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(1)

                .build();


        Action closeMiddle = closeBot.getDrive().actionBuilder(closeStarting)
                .strafeTo(new Vector2d(12, 31))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(51, 36, Math.toRadians(180)), Math.toRadians(0))

                .waitSeconds(servoWaitTime) // score
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(20, 11, Math.toRadians(180)), Math.toRadians(180))

                .splineToSplineHeading(new Pose2d(-20, 10.75, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59, 11.125, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(0, 11, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(20, 11, Math.toRadians(225)), Math.toRadians(0))

                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(28, 11), Math.toRadians(180))
//                .setTangent(Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(
                        41, 36, Math.toRadians(180)
                ), Math.toRadians(90))

//                .splineTo(new Vector2d(-59, 11.125), Math.toRadians(0))
//                .waitSeconds(5)

                .build();

        Action farMiddle = farBot.getDrive().actionBuilder(farStarting)

                .strafeToConstantHeading(new Vector2d(-46, 35))
                .strafeTo(new Vector2d(-46, 41))
                .turnTo(Math.toRadians(180))

                .strafeToConstantHeading(new Vector2d(-59.5, 35.25))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-51.5, 35.25))

                .waitSeconds(servoWaitTime)
                .strafeTo(new Vector2d(-59.5, 35.25))
                .waitSeconds(2)

                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-40, 58, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(
                        new Vector2d(-40, 58),
                        Math.toRadians(0)
                )
                .strafeTo(new Vector2d(0, 58))
                .strafeToSplineHeading(new Vector2d(28, 58), Math.toRadians(135))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(38, 41), Math.toRadians(180))
//                .setTangent(Math.toRadians(315))
//                .splineToSplineHeading(new Pose2d(40, 41, Math.toRadians(180)), Math.toRadians(0))
//                .strafeTo(new Vector2d(51, 41))
                .waitSeconds(0.0001)
                .strafeTo(new Vector2d(51, 43.25))
                .waitSeconds(servoWaitTime)

                .build();

//        closeBot.runAction(closeMiddle);
        farBot.runAction(farMiddle);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(closeBot)
                .addEntity(farBot)
                .start();
    }
}