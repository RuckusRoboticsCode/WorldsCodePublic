package org.firstinspires.ftc.teamcode.Vision.AprilTag;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagPoseSolver {

    public static Pose2d getRobotPoseBackdrop(
            ArrayList<AprilTagDetection> detections,
            Pose2d poseEstimate,
            Vector2d allowableError) {
        ArrayList<Vector2d> robotPositions = new ArrayList<>();
        double headingRad = poseEstimate.heading.toDouble();

        if (detections != null && detections.size() > 0) {

            for (AprilTagDetection detection : detections) {

                if (detection.id > 6 || detection.id < 0) {
                    continue;
                }

                if (detection.metadata == null) {
                    continue;
                }

                double relativeX = detection.ftcPose.x - Camera.lateralOffset;
                double relativeY = detection.ftcPose.y - Camera.forwardOffset;
//
//                double x = detection.ftcPose.x - Camera.forwardOffset;
//                double y = detection.ftcPose.y - Camera.lateralOffset;
                double rotationAngle = -headingRad;
//
//                double x2 = x * Math.cos(rotationAngle) + y * Math.sin(rotationAngle);
//                double y2 = x * -Math.sin(rotationAngle) + y * Math.cos(rotationAngle);
//
                Vector2d fieldCentricOffset = new Vector2d(
                        -relativeX * Math.sin(rotationAngle) + relativeY * Math.cos(rotationAngle),
                        -(relativeX * Math.cos(rotationAngle) + relativeY * Math.sin(rotationAngle))
                );

                VectorF tempTagPose = CenterStageTagLibrary.getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
                if (tempTagPose == null) {
                    continue;
                }

                Vector2d tagPose = convertVector(tempTagPose);
//
//                Vector2d robotPos = new Vector2d(
//                        tempTagPose.get(0) + y2,
//                        tempTagPose.get(1) - x2
//                );

                Vector2d robotPos = tagPose.plus(fieldCentricOffset);

                addPoseEstimate(
                        poseEstimate.position,
                        robotPos,
                        allowableError,
                        robotPositions
                );
            }
        }

        Vector2d averagePosition = getAverageVector(robotPositions);

        return new Pose2d(
                averagePosition,
                headingRad
        );
    }

    public static Pose2d getRobotPoseBackdrop(
            ArrayList<AprilTagDetection> detections,
            Pose2d poseEstimate) {

        return getRobotPoseBackdrop(detections, poseEstimate, new Vector2d(1000, 1000));
    }

    public static Pose2d getRobotPoseBackdrop(
            Supplier<ArrayList<AprilTagDetection>> arrayListSupplier,
            Supplier<Pose2d> poseSupplier,
            Vector2d allowableError) {

        return getRobotPoseBackdrop(arrayListSupplier.get(), poseSupplier.get(), allowableError);
    }

    public static Pose2d getRobotPoseBackdrop(
            Supplier<ArrayList<AprilTagDetection>> arrayListSupplier,
            Supplier<Pose2d> poseSupplier) {

        return getRobotPoseBackdrop(arrayListSupplier.get(), poseSupplier.get());
    }

    private static Vector2d convertVector(VectorF vec) {
        return new Vector2d(vec.get(0), vec.get(1));
    }

    private static Vector2d getAverageVector(ArrayList<Vector2d> vector2ds) {
        Vector2d averageVector = new Vector2d(0, 0);
        for (Vector2d v : vector2ds) {
            averageVector = averageVector.plus(v);
        }
        averageVector = averageVector.div(vector2ds.size());
        return averageVector;
    }

    private static void addPoseEstimate(Vector2d currentPose, Vector2d newPose, Vector2d allowableError, ArrayList<Vector2d> poses) {
        if (Math.abs(currentPose.x - newPose.x) > allowableError.x) {
            return;
        }

        if (Math.abs(currentPose.y - newPose.y) > allowableError.y) {
            return;
        }

        poses.add(newPose);

    }
}