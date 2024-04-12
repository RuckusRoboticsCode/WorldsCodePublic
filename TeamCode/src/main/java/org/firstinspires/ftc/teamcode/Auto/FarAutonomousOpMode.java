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

import org.firstinspires.ftc.teamcode.Auto.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.Auto.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Auto.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Auto.Util.Parking;
import org.firstinspires.ftc.teamcode.Auto.Util.Path;
import org.firstinspires.ftc.teamcode.Helper.Caching.IntermittentVoltageSensor;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.LinearOpModeEx;
import org.firstinspires.ftc.teamcode.Vision.Prop.Prop;

import java.util.ArrayList;

public class FarAutonomousOpMode {

    public LinearSlides linearSlides;
    public Deposit deposit;
    public Intake intake;
    public Vision vision;
    public MecanumDrive drive;

    private final AllianceColor allianceColor;
    private final Path path;
    private final Parking parking;
    private int numCycles = 0;

    private Pose2d startingPose;

    private final int preLoadHeight = 800;
    private final int firstCycleHeight = 1200;
    private final int secondCycleHeight = 1500;

    private final double depositTime = 0.5;
    private final double resetWaitTime = 0.5;

    private final double rakePullDistance = 8.0;
    private final double intakeTimeout = 2.5;
    private Prop.Location propLocation;

    private final ArrayList<Action> preLoadActions = new ArrayList<>();
    private final ArrayList<Action> parkActions = new ArrayList<>();
    private final ArrayList<Action> firstCycleActions = new ArrayList<>();
    private Action stageDoorFirstCycleIntakeAction;
    private Action trussFirstCycleIntakeAction;
    private Action trussStackToWait;
    private Action stageDoorStackToWait;
    private Action scoreAction;

    private Action preLoad;
    private Action park;
    private Action firstCycle;

    private final Action resetAction;
    private final Action scorePreLoadAction;

    public FarAutonomousOpMode(LinearOpModeEx linearOpModeEx, AllianceColor allianceColor, Parking parking, Path path, int numCycles) {

        this.parking = parking;
        this.allianceColor = allianceColor;
        this.path = path;
        this.vision = new Vision(linearOpModeEx, allianceColor, true);
        this.numCycles = numCycles;

        linearSlides = new LinearSlides(linearOpModeEx);
        deposit = new Deposit(linearOpModeEx);
        intake = new Intake(linearOpModeEx);

        startingPose = BlueLocations.Poses.FAR_STARTING.pose;
        if (allianceColor == AllianceColor.RED) {
            startingPose = new Pose2d(
                    new Vector2d(startingPose.position.x, -startingPose.position.y),
                    startingPose.heading.inverse()
            );
        }

        drive = new MecanumDrive(linearOpModeEx.hardwareMap, startingPose);
        linearSlides.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());
        intake.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());

        resetAction = new SequentialAction(
                new ParallelAction(
                    linearSlides.moveTo(0),
                    deposit.flipGate(Deposit.GatePositions.CLOSED),
                    deposit.moveDeposit(Deposit.DepositPositions.INTAKE)
                ),
                new SleepAction(resetWaitTime)
        );

        scorePreLoadAction = new SequentialAction(
                new ParallelAction(
                        linearSlides.moveTo(preLoadHeight),
                        deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE),
                        drive.scoreBackdrop(
                                allianceColor, vision::getPixelLocation, this::getStoredPropLocation
                        )
                ),
                scoreAction
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
                        intake.intakeTwo(Intake.IntakeSpeeds.MEDIUM_INTAKE, intakeTimeout),
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
                        intake.intakeTwo(Intake.IntakeSpeeds.MEDIUM_INTAKE, intakeTimeout),
                        drive.actionBuilder(
                                        new Pose2d(BlueLocations.Poses.TRUSS_STACK.vector.plus(new Vector2d(rakePullDistance, 0)), Math.toRadians(180)), getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.TRUSS_STACK.vector,
                                        new TranslationalVelConstraint(25),
                                        new ProfileAccelConstraint(-30, 30))
                                .build()
                )
        );

        trussStackToWait = drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.pose, Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.CYCLE_START_TRUSS.vector.y))
                .strafeToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(135))
                .build();

        stageDoorStackToWait = drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
//                .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y))
//                .strafeToSplineHeading(
//                        BlueLocations.Poses.STAGE_DOOR_WAIT.vector.plus(new Vector2d(7, 0)),
//                        Math.toRadians(225))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(
                        0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y, Math.toRadians(180)
                ), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(
                        BlueLocations.Poses.STAGE_DOOR_WAIT.vector.plus(new Vector2d(7, 0)),
                        Math.toRadians(225)
                ), Math.toRadians(180))
                .build();

        scoreAction = new SequentialAction(
                deposit.flipGate(Deposit.GatePositions.OPEN),
                new SleepAction(depositTime)
        );

        buildAll();
    }

    public FarAutonomousOpMode(LinearOpModeEx linearOpModeEx, AllianceColor allianceColor, Parking parking, Path path) {
        this.parking = parking;
        this.allianceColor = allianceColor;
        this.path = path;
        this.vision = new Vision(linearOpModeEx, allianceColor, true);

        linearSlides = new LinearSlides(linearOpModeEx);
        deposit = new Deposit(linearOpModeEx);
        intake = new Intake(linearOpModeEx);

        startingPose = BlueLocations.Poses.FAR_STARTING.pose;
        if (allianceColor == AllianceColor.RED) {
            startingPose = new Pose2d(
                    new Vector2d(startingPose.position.x, -startingPose.position.y),
                    startingPose.heading.inverse()
            );
        }

        drive = new MecanumDrive(linearOpModeEx.hardwareMap, startingPose);
        linearSlides.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());
        intake.setIntermittentVoltageSensor(drive.getIntermittentVoltageSensor());

        resetAction = new SequentialAction(
                new SleepAction(resetWaitTime),
                new ParallelAction(
                        linearSlides.moveTo(0),
                        deposit.flipGate(Deposit.GatePositions.CLOSED)
                ),
                deposit.moveDeposit(Deposit.DepositPositions.INTAKE)
        );

        scorePreLoadAction = new SequentialAction(
                new ParallelAction(
                    linearSlides.moveTo(preLoadHeight),
                    deposit.moveDeposit(Deposit.DepositPositions.OUTTAKE),
                    drive.scoreBackdrop(
                            allianceColor, vision::getPixelLocation, this::getStoredPropLocation
                    )
                ),
                scoreAction
        );

        scoreAction = new SequentialAction(
                deposit.flipGate(Deposit.GatePositions.OPEN),
                new SleepAction(depositTime)
        );

        buildAll();
    }

    public Prop.Location detectPropLocation() {
        propLocation = vision.getPropLocation();

        preLoad = getTrajectory(preLoadActions, propLocation);
        park = getTrajectory(parkActions, propLocation);

        if (numCycles != 0) {
            firstCycle = getTrajectory(firstCycleActions, propLocation);
        }

        return propLocation;
    }

    private Prop.Location getStoredPropLocation() {
        return propLocation;
    }

    public Action getPreLoad() {
        return preLoad;
    }

    public Action getFirstCycle() {
        return firstCycle;
    }

    public Action getPark() {
        return park;
    }

    private void buildAll() {
        buildPreLoad();
        buildParking();

        if (numCycles != 0) {
            buildFirstCycle();
        }
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
        if (path == Path.STAGE_DOOR) {
            buildPreLoadStageDoor();
        } else {
            buildPreLoadTruss();
        }
    }

    private void buildPreLoadStageDoor() {
        // Blue: left = 0
        // Red: right = 0

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToSplineHeading(new Vector2d(-38, 43), Math.toRadians(115))
                                .splineToLinearHeading(new Pose2d(BlueLocations.Poses.SPIKE_MARK_LEFT_FAR.vector, Math.toRadians(135)), Math.toRadians(315))

                                .strafeTo(new Vector2d(-39, 35))
                                .splineToSplineHeading(BlueLocations.Poses.STAGE_DOOR_STACK.pose, Math.toRadians(180))

                                .build(),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.STAGE_DOOR_WAIT.vector.plus(new Vector2d(7, 0)), Math.toRadians(225)), getPoseMap())
                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_LEFT.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .build(),
                        scorePreLoadAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.SPIKE_MARK_MIDDLE_FAR.vector)

                                .strafeToConstantHeading(new Vector2d(-54, 35))
                                .strafeToLinearHeading(new Vector2d(-54, BlueLocations.Poses.STAGE_DOOR_STACK.vector.y), Math.toRadians(180))
                                .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector)

                                .build(),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.STAGE_DOOR_WAIT.vector.plus(new Vector2d(7, 0)), Math.toRadians(225)), getPoseMap())
                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_MIDDLE.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .build(),
                        scorePreLoadAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.SPIKE_MARK_RIGHT_FAR.vector)
                                .strafeToConstantHeading(new Vector2d(-34, 42))
                                .strafeToConstantHeading(new Vector2d(-34, BlueLocations.Poses.STAGE_DOOR_STACK.vector.y))
                                .strafeToLinearHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector, Math.toRadians(180))

                                .build(),
                        stageDoorFirstCycleIntakeAction,
                        stageDoorStackToWait,
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.STAGE_DOOR_WAIT.vector.plus(new Vector2d(7, 0)), Math.toRadians(225)), getPoseMap())
                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_RIGHT.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .build(),
                        scorePreLoadAction
                )
        );
    }

    private void buildPreLoadTruss() {
        // Blue: left = 0
        // Red: right = 0

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToSplineHeading(new Vector2d(-38, 43), Math.toRadians(115))
                                .splineToLinearHeading(new Pose2d(BlueLocations.Poses.SPIKE_MARK_LEFT_FAR.vector, Math.toRadians(135)), Math.toRadians(315))

                                .strafeTo(new Vector2d(-39, 35))
                                .splineToSplineHeading(BlueLocations.Poses.TRUSS_STACK.pose, Math.toRadians(180))

                                .build(),
                        trussFirstCycleIntakeAction,
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135)), getPoseMap())
//                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_LEFT.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .strafeToSplineHeading(new Vector2d(
                                        40, BlueLocations.Poses.BACKDROP_LEFT.vector.y), Math.toRadians(180)
                                )
                                .build(),
                        scorePreLoadAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.SPIKE_MARK_MIDDLE_FAR.vector)
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(BlueLocations.Poses.TRUSS_STACK.pose, Math.toRadians(180))

                                .build(),
                        trussFirstCycleIntakeAction,
                        trussStackToWait,
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135)), getPoseMap())
//                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_MIDDLE.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .strafeToSplineHeading(new Vector2d(
                                        40, BlueLocations.Poses.BACKDROP_MIDDLE.vector.y), Math.toRadians(180)
                                )
                                .build(),
                        scorePreLoadAction
                )
        );

        preLoadActions.add(
                new SequentialAction(
                        drive.actionBuilder(startingPose, getPoseMap())
                                .strafeToConstantHeading(BlueLocations.Poses.SPIKE_MARK_RIGHT_FAR.vector)
                                .strafeToConstantHeading(new Vector2d(-46, 35))
                                .strafeTo(new Vector2d(-46, 41))
                                .turnTo(Math.toRadians(180))
                                .strafeToConstantHeading(BlueLocations.Poses.TRUSS_STACK.vector)

                                .build(),
                        trussFirstCycleIntakeAction,
                        drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(
                                        BlueLocations.Poses.CYCLE_START_TRUSS_2.vector.plus(new Vector2d(-10, 0)),
                                        BlueLocations.Poses.CYCLE_START_TRUSS_2.pose.heading.toDouble()
                                ), Math.toRadians(0))
                                .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.CYCLE_START_TRUSS.vector.y))
                                .strafeToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS.vector, Math.toRadians(135))
                                .build(),
                        vision.waitForBackdropClear(),
                        drive.actionBuilder(new Pose2d(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135)), getPoseMap())
//                                .strafeToSplineHeading(BlueLocations.Poses.BACKDROP_RIGHT.vector.plus(new Vector2d(-5, 0)), Math.toRadians(180))
                                .strafeToSplineHeading(new Vector2d(
                                        40, BlueLocations.Poses.BACKDROP_RIGHT.vector.y), Math.toRadians(180)
                                )
                                .build(),
                        scorePreLoadAction
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
                                        .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225))
                                        .build()
                        ),
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
                                        .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225))
                                        .build()
                        ),
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
                                        .strafeToConstantHeading(BlueLocations.Poses.STAGE_DOOR_STACK.vector)
                                        .build(),
                                resetAction
                        ),
                        stageDoorFirstCycleIntakeAction,
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.STAGE_DOOR_STACK.pose, getPoseMap())
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.STAGE_DOOR_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.STAGE_DOOR_WAIT.vector, Math.toRadians(225))
                                        .build()
                        ),
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
                        // intake action
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                                        .setTangent(Math.toRadians(0))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.pose, Math.toRadians(0))
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.TRUSS_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135))
                                        .build()
                        ),
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
                        // intake action
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                                        .setTangent(Math.toRadians(0))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.pose, Math.toRadians(0))
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.TRUSS_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135))
                                        .build()
                        ),
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
                        // intake action
                        new ParallelAction(
                                intake.runIntake(Intake.IntakeSpeeds.OUTTAKE, 1.5),
                                drive.actionBuilder(BlueLocations.Poses.TRUSS_STACK.pose, getPoseMap())
                                        .setTangent(Math.toRadians(0))
                                        .splineToSplineHeading(BlueLocations.Poses.CYCLE_START_TRUSS_2.pose, Math.toRadians(0))
                                        .strafeToConstantHeading(new Vector2d(0, BlueLocations.Poses.TRUSS_WAIT.vector.y))
                                        .strafeToSplineHeading(BlueLocations.Poses.TRUSS_WAIT.vector, Math.toRadians(135))
                                        .build()
                        ),
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
}
