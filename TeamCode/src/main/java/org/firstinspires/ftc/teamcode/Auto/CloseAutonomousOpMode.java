package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Auto.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;

import org.firstinspires.ftc.teamcode.Auto.Util.*;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

import java.util.ArrayList;

public class CloseAutonomousOpMode {

    public LinearSlides linearSlides;
    public Deposit deposit;
    public Intake intake;
    public Vision vision;
    public MecanumDrive drive;

    private final AllianceColor allianceColor;
    private Path path = Path.TRUSS;
    private final Parking parking;
    private int numCycles = 0;

    private Pose2d startingPose;

    private final int preLoadHeight = 750;
    private final int firstCycleHeight = 1200;
    private final int secondCycleHeight = 1500;

    private final double depositTime = 0.75;
    private final double resetWaitTime = 0.5;

    private final double rakePullDistance = 8.0;
    private final double intakeTimeout = 2.5;

    private final ArrayList<Action> preLoadActions = new ArrayList<>();
    private final ArrayList<Action> parkActions = new ArrayList<>();
    private final ArrayList<Action> firstCycleActions = new ArrayList<>();
    private final ArrayList<Action> secondCycleActions = new ArrayList<>();
    private Action stageDoorFirstCycleIntakeAction;
    private Action trussFirstCycleIntakeAction;
    private Action stageDoorStackToWait;
    private Action trussStackToWait;
    private Action scoreAction;

    private Action preLoad;
    private Action park;
    private Action firstCycle;
    private Action secondCycle;

    private Action resetAction;

    public CloseAutonomousOpMode(LinearOpModeEx linearOpModeEx, AllianceColor allianceColor, Parking parking, Path path, int numCycles) {

        this.parking = parking;
        this.allianceColor = allianceColor;
        this.vision = new Vision(linearOpModeEx, allianceColor, true);
        this.path = path;
        this.numCycles = numCycles;

        linearSlides = new LinearSlides(linearOpModeEx);
        deposit = new Deposit(linearOpModeEx);
        intake = new Intake(linearOpModeEx);

        startingPose = BlueLocations.Poses.CLOSE_STARTING.pose;
        if (allianceColor == AllianceColor.RED) {
            startingPose = new Pose2d(
                    new Vector2d(startingPose.position.x, -startingPose.position.y),
                    startingPose.heading.inverse()
            );
        }

        drive = new MecanumDrive(linearOpModeEx.hardwareMap, startingPose);
        linearSlides.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());
        intake.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());

        buildCommonActions();
        buildAll();
    }

    public CloseAutonomousOpMode(LinearOpModeEx linearOpModeEx, AllianceColor allianceColor) {
        this(linearOpModeEx, allianceColor, Parking.CORNER, Path.TRUSS, 0);
    }

    private void buildCommonActions() {
        scoreAction = new SequentialAction(
                deposit.flipGate(Deposit.GatePositions.OPEN),
                new SleepAction(depositTime)
        );

        resetAction = new SequentialAction(
                new SleepAction(resetWaitTime),
                new ParallelAction(
                        linearSlides.moveTo(0),
                        deposit.flipGate(Deposit.GatePositions.CLOSED)
                ),
                deposit.moveDeposit(Deposit.DepositPositions.INTAKE)
        );

        stageDoorFirstCycleIntakeAction = new SequentialAction(
                intake.moveRake(Intake.RakePositions.INTAKE_TWO),
                new SleepAction(depositTime),
                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
                        .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector.plus(new Vector2d(rakePullDistance, 0)))
                        .build(),
                intake.moveRake(Intake.RakePositions.LIFTED),
                new SleepAction(depositTime),
                new ParallelAction(
                        intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, intakeTimeout),
                        drive.actionBuilder(
                                        new Pose2d(BlueLocations.Poses.STAGE_DOOR_STACK.vector.plus(new Vector2d(rakePullDistance, 0)), Math.toRadians(180)), getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector,
                                        new TranslationalVelConstraint(25),
                                        new ProfileAccelConstraint(-30, 30))
                                .build()
                )
        );

        trussFirstCycleIntakeAction = new SequentialAction(
                intake.moveRake(Intake.RakePositions.INTAKE_TWO),
                new SleepAction(depositTime),
                drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                        .strafeToConstantHeading(BlueLocations.Poses.TRUSS_STACK.vector.plus(new Vector2d(rakePullDistance, 0)))
                        .build(),
                intake.moveRake(Intake.RakePositions.LIFTED),
                new SleepAction(depositTime),
                new ParallelAction(
                        intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, intakeTimeout),
                        drive.actionBuilder(
                                        new Pose2d(BlueLocations.Poses.TRUSS_STACK.vector.plus(new Vector2d(rakePullDistance, 0)), Math.toRadians(180)), getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.TRUSS_STACK.vector,
                                        new TranslationalVelConstraint(25),
                                        new ProfileAccelConstraint(-30, 30))
                                .build()
                )
        );

        stageDoorStackToWait = new ParallelAction(
                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y))
                        .strafeToSplineHeading(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225))
                        .build()
        );

        trussStackToWait = new ParallelAction(
                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.pose, Math.toRadians(0))
                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.TRUSS_WAIT.vector.y))
                        .strafeToSplineHeading(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135))
                        .build()
        );
    }

    public Prop.Location detectPropLocation() {
        Prop.Location propLocation = vision.getPropLocation();

        preLoad = getTrajectory(preLoadActions, propLocation);
        park = getTrajectory(parkActions, propLocation);

        if (numCycles != 0) {
            firstCycle = getTrajectory(firstCycleActions, propLocation);
        }

        if (numCycles == 2) {
            secondCycle = getTrajectory(secondCycleActions, propLocation);
        }

//        vision.closePropProcessor();

        return propLocation;
    }

    public Action getPreLoad() {
        return preLoad;
    }

    public Action getPark() {
        return park;
    }

    public Action getFirstCycle() {
        return firstCycle;
    }

    public Action getSecondCycle() {return secondCycle;}

    private void buildAll() {
        buildPreLoad();
        buildParking();

        if (numCycles == 0) {
            return;
        }

        buildFirstCycle();

        if (numCycles == 1) {
            return;
        }

        buildSecondCycle();
    }

    public void runAll() {
        Actions.runBlocking(preLoad);
        if (numCycles == 0) {
            Actions.runBlocking(park);
            return;
        }
        Actions.runBlocking(firstCycle);
        if (numCycles == 1) {
            Actions.runBlocking(park);
            return;
        }
        Actions.runBlocking(secondCycle);
        Actions.runBlocking(park);
    }

    private Action getTrajectory(ArrayList<Action> trajectories, Prop.Location propLocation) {
        switch (propLocation) {
            case LEFT:
                return trajectories.get(
                        allianceColor == AllianceColor.RED ? 2 : 0
                );
            case MIDDLE:
                return trajectories.get(1);
            case RIGHT:
                return trajectories.get(
                        allianceColor == AllianceColor.RED ? 0 : 2
                );
        }
        return null;
    }

    private PoseMap getPoseMap() {
        return allianceColor == AllianceColor.RED ?
                pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse()) :
                pose -> new Pose2dDual<>(pose.position.x, pose.position.y, pose.heading);
    }

    private void buildPreLoad() {
        // Blue: left = 0
        // Red: right = 0

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.SPIKE_MARK_LEFT_CLOSE.vector)
                                .afterTime(0, linearSlides.moveTo(preLoadHeight))
                                .strafeToConstantHeading(new Vector2d(26, 46))

                                .strafeToSplineHeading(
                                        new Vector2d(28, 50),
                                        Math.toRadians(135)
                                )

                                .splineToLinearHeading(
                                        BlueLocations.Poses.BACKDROP_LEFT.pose,
                                        Math.toRadians(0)
                                )
                                .afterTime(0, deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE))
                                .build(),
                        scoreAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.SPIKE_MARK_MIDDLE_CLOSE.vector)
                                .setTangent(Math.toRadians(90))
                                .afterTime(0, linearSlides.moveTo(preLoadHeight))
                                .splineToLinearHeading(
                                        BlueLocations.Poses.BACKDROP_MIDDLE.pose,
                                        Math.toRadians(0)
                                )
                                .afterTime(0, deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE))
                                .build(),
                        scoreAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeTo(new Vector2d(15, 54))
                                .splineTo(BlueLocations.Poses.SPIKE_MARK_RIGHT_CLOSE.vector, Math.toRadians(225))
                                .afterTime(0, linearSlides.moveTo(preLoadHeight))
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(
                                        BlueLocations.Poses.BACKDROP_RIGHT.pose,
                                        Math.toRadians(0)
                                )
                                .afterTime(0, deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE))
                                .build(),
                        scoreAction
                )
        );
    }

    private void buildParking() {
        if (parking == Parking.MIDDLE) {
            buildParkingMiddle();
            return;
        }
        buildParkingCorner();
    }

    private void buildParkingMiddle() {
        // Blue: left = 0
        // Red: right = 0

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(44, 23), Math.toRadians(270))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_MIDDLE.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(44, 23), Math.toRadians(270))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_MIDDLE.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(44, 23), Math.toRadians(270))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_MIDDLE.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );
    }

    private void buildParkingCorner() {
        // Blue: left = 0
        // Red: right = 0

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(42, 49), Math.toRadians(90))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_CORNER.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(42, 49), Math.toRadians(90))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_CORNER.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );

        parkActions.add(
                new ParallelAction(
                    drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(42, 49), Math.toRadians(90))
                            .splineToConstantHeading(BlueLocations.Poses.PARKING_CORNER.vector, Math.toRadians(0))
                            .build(),
                    resetAction
                )
        );
    }

    private void buildFirstCycle() {
        if (path == Path.STAGE_DOOR) {
            buildStageDoorFirstCycle();
            return;
        }
        buildTrussFirstCycle();
    }

    private void buildStageDoorFirstCycle() {
        // Blue: left = 0
        // Red: right = 0

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                            drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                                    .setTangent(Math.toRadians(180))
                                    .splineTo(BlueLocations.Poses.CYCLE_START_STAGE_DOOR.vector, Math.toRadians(180))
                                    .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_STAGE_DOOR_2.pose, Math.toRadians(180))
                                    .splineToSplineHeading(BlueLocations.Poses.STAGE_DOOR_STACK.pose, Math.toRadians(180))
                                    .build(),
                            resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_WAIT.pose, getPoseMap())
                                        .strafeToLinearHeading(BlueLocations.Poses.BACKDROP_RIGHT.vector, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                            drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                    .setTangent(Math.toRadians(180))
                                    .splineTo(BlueLocations.Poses.CYCLE_START_STAGE_DOOR.vector, Math.toRadians(180))
                                    .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_STAGE_DOOR_2.pose, Math.toRadians(180))
                                    .splineToSplineHeading(BlueLocations.Poses.STAGE_DOOR_STACK.pose, Math.toRadians(180))
                                    .build(),
                            resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_WAIT.pose, getPoseMap())
                                        .strafeToLinearHeading(BlueLocations.Poses.BACKDROP_RIGHT.vector, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineTo(BlueLocations.Poses.CYCLE_START_STAGE_DOOR.vector, Math.toRadians(180))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_STAGE_DOOR_2.pose, Math.toRadians(180))
                                        .splineToSplineHeading(BlueLocations.Poses.STAGE_DOOR_STACK.pose, Math.toRadians(180))
                                        .build(),
                                resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_WAIT.pose, getPoseMap())
                                        .strafeToLinearHeading(BlueLocations.Poses.BACKDROP_MIDDLE.vector, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );
    }

    private void buildTrussFirstCycle() {
        // Blue: left = 0
        // Red: right = 0

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS.pose, Math.toRadians(180))
                                        .strafeTo(BlueLocations.Poses.CYCLE_START_TRUSS_2.vector)
                                        .splineToSplineHeading(BlueLocations.Poses.TRUSS_STACK.pose, Math.toRadians(180))
                                        .build(),
                                resetAction
                        ),
                        trussFirstCycleIntakeAction,
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_WAIT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(315))
                                        .splineToSplineHeading(BlueLocations.Poses.BACKDROP_MIDDLE.pose, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineTo(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(180))
                                        .strafeToConstantHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.vector)
                                        .splineToSplineHeading(BlueLocations.Poses.TRUSS_STACK.pose, Math.toRadians(180))
                                        .build(),
                                resetAction
                        ),
                        trussFirstCycleIntakeAction,
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_WAIT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(315))
                                        .splineToSplineHeading(BlueLocations.Poses.BACKDROP_LEFT.pose, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );

        firstCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineTo(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(180))
                                        .strafeToConstantHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.vector)
                                        .splineToSplineHeading(BlueLocations.Poses.TRUSS_STACK.pose, Math.toRadians(180))
                                        .build(),
                                resetAction
                        ),
                        trussFirstCycleIntakeAction,
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_WAIT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(315))
                                        .splineToSplineHeading(BlueLocations.Poses.BACKDROP_LEFT.pose, Math.toRadians(180))
                                        .build(),
                                linearSlides.moveTo(firstCycleHeight),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction
                )
        );
    }

    private void buildSecondCycle() {
        if (path == Path.STAGE_DOOR) {
            buildStageDoorSecondCycle();
            return;
        }
        buildTrussSecondCycle();
    }

    private void buildStageDoorSecondCycle() {
        // Blue: left = 0
        // Red: right = 0

        secondCycleActions.add(

                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_STAGE_DOOR.pose, Math.toRadians(180))
                                        .strafeTo(BlueLocations.Poses.STAGE_DOOR_STACK.vector.plus(new Vector2d(8, 0)))
                                        .afterTime(0, intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, 1.5))
                                        .strafeTo(BlueLocations.Poses.STAGE_DOOR_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                linearSlides.moveTo(secondCycleHeight),
                                drive.actionBuilder(new Pose2d(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225)), getPoseMap())
                                        .setTangent(Math.toRadians(45))
                                        .splineTo(BlueLocations.Poses.BACKDROP_RIGHT.vector, Math.toRadians(0))
                                        .build(),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction,
                        drive.actionBuilder(BlueLocations.Poses.BACKDROP_RIGHT.pose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.BACKDROP_RIGHT.vector.plus(new Vector2d(-5, 0)))
                                .build(),
                        resetAction
                )
        );

        secondCycleActions.add(secondCycleActions.get(0));

        secondCycleActions.add(

                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_STAGE_DOOR.pose, Math.toRadians(180))
                                        .strafeTo(BlueLocations.Poses.STAGE_DOOR_STACK.vector.plus(new Vector2d(8, 0)))
                                        .afterTime(0, intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, 1.5))
                                        .strafeTo(BlueLocations.Poses.STAGE_DOOR_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                linearSlides.moveTo(secondCycleHeight),
                                drive.actionBuilder(new Pose2d(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225)), getPoseMap())
                                        .setTangent(Math.toRadians(45))
                                        .splineTo(BlueLocations.Poses.BACKDROP_MIDDLE.vector, Math.toRadians(0))
                                        .build(),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction,
                        drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.BACKDROP_MIDDLE.vector.plus(new Vector2d(-5, 0)))
                                .build(),
                        resetAction
                )
        );
    }

    private void buildTrussSecondCycle() {
        // Blue: left = 0
        // Red: right = 0

        secondCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(180)), Math.toRadians(180))
                                        .strafeTo(BlueLocations.Poses.CYCLE_START_TRUSS_2.vector)
                                        .splineTo(BlueLocations.Poses.TRUSS_STACK.vector.plus(new Vector2d(8, 0)), Math.toRadians(180))
                                        .afterTime(0, intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, 1.5))
                                        .strafeTo(BlueLocations.Poses.TRUSS_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                linearSlides.moveTo(secondCycleHeight),
                                drive.actionBuilder(new Pose2d(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135)), getPoseMap())
                                        .setTangent(Math.toRadians(315))
                                        .splineTo(BlueLocations.Poses.BACKDROP_MIDDLE.vector, Math.toRadians(0))
                                        .build(),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction,
                        drive.actionBuilder(BlueLocations.Poses.BACKDROP_MIDDLE.pose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.BACKDROP_MIDDLE.vector.plus(new Vector2d(-5, 0)))
                                .build(),
                        resetAction
                )
        );

        secondCycleActions.add(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                                        .setTangent(Math.toRadians(180))
                                        .splineToSplineHeading(new Pose2d(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(180)), Math.toRadians(180))
                                        .strafeTo(BlueLocations.Poses.CYCLE_START_TRUSS_2.vector)
                                        .splineTo(BlueLocations.Poses.TRUSS_STACK.vector.plus(new Vector2d(8, 0)), Math.toRadians(180))
                                        .afterTime(0, intake.intakeTwo(Intake.IntakeSpeeds.FAST_INTAKE, 1.5))
                                        .strafeTo(BlueLocations.Poses.TRUSS_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        new ParallelAction(
                                linearSlides.moveTo(secondCycleHeight),
                                drive.actionBuilder(new Pose2d(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135)), getPoseMap())
                                        .setTangent(Math.toRadians(315))
                                        .splineTo(BlueLocations.Poses.BACKDROP_LEFT.vector, Math.toRadians(0))
                                        .build(),
                                deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE)
                        ),
                        scoreAction,
                        drive.actionBuilder(BlueLocations.Poses.BACKDROP_LEFT.pose, getPoseMap())
                                .strafeTo(BlueLocations.Poses.BACKDROP_LEFT.vector.plus(new Vector2d(-5, 0)))
                                .build(),
                        resetAction
                )
        );

        secondCycleActions.add(secondCycleActions.get(1));

    }

    public void close() {
        vision.closeAll();
    }
}
