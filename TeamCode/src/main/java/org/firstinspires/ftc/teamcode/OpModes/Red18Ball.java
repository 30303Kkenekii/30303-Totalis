package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.LEDSubsystem;
import org.firstinspires.ftc.teamcode.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "🟥🟥🟥Red15BallAuto🟥🟥🟥", group = "Autonomous")
public class Red18Ball extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime shooterGateTimer;
    private ElapsedTime gateTimer;
    private ElapsedTime intakeTimer;

    // --- COORDINATES ---
    private final Pose startPose = new Pose(125.3233, 115.5557, Math.toRadians(37.8449));
    private final Pose scorePose = new Pose(90, 84.3, Math.toRadians(29));
    private final Pose firstSpikeMarkPose = new Pose(127.4342, 70, Math.toRadians(0));
    private final Pose secondSpikeMarkPose = new Pose(136, 54, Math.toRadians(-0));
    private final Pose thirdSpikeMarkPose = new Pose(135, 30.3274, Math.toRadians(0));
    private final Pose setUpForthPickupPose = new Pose(136.3332, 54, Math.toRadians(-90));
    private final Pose forthPickupPose = new Pose(135, 5, Math.toRadians(-85));
    private final Pose endPose = new Pose(118, 66, Math.toRadians(0));
    public static Pose gatePose = new Pose(127, 62, Math.toRadians(0));
    public static Pose gatePoseControlPoint = new Pose(110, 50);
    public static Pose secondControlPoint = new Pose(80, 62);
    public static Pose returnSecondControlPoint = new Pose(110, 55);
    public static Pose thirdControlPoint = new Pose(80, 38);
    public static Pose returnThirdControlPoint = new Pose(110, 36);

    public static double shootTime = .5;
    public static double shooterGateOpenTime = .3;
    public static double intakeTime = .3;
    public static double firstSpikeWaitTime = 0.8;
    public static double gateWaitTime = 1;
    public static double fourthPickupWaitTime = 4;
    public static double fourthPickupSlowdownPoint = 0.9;
    public static double fourthPickupSlowSpeed = 0.5;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;

    private PathChain scorePreload, grabFirstSpikemark, scoreFirstSpikemark,
            grabSecondSpikemark, secondGateOpen, scoreSecondSpikemark,
            grabThirdSpikemark, scoreThirdSpikemark, grabFourthPickup, scoreFourthPickup, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabFirstSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, firstSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstSpikeMarkPose.getHeading())
                .build();

        scoreFirstSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikeMarkPose, scorePose))
                .setLinearHeadingInterpolation(firstSpikeMarkPose.getHeading(), scorePose.getHeading())
                .build();

        grabSecondSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondControlPoint, secondSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondSpikeMarkPose.getHeading())
                .build();

        secondGateOpen = follower.pathBuilder()
                .addPath(new BezierCurve(secondSpikeMarkPose, gatePoseControlPoint, gatePose))
                .setLinearHeadingInterpolation(secondSpikeMarkPose.getHeading(), gatePose.getHeading())
                .build();

        scoreSecondSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        grabThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, thirdControlPoint, thirdSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdSpikeMarkPose.getHeading())
                .build();

        scoreThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikeMarkPose, scorePose))
                .setLinearHeadingInterpolation(thirdSpikeMarkPose.getHeading(), scorePose.getHeading())
                .build();

        grabFourthPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondControlPoint, setUpForthPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), setUpForthPickupPose.getHeading())
                .addPath(new BezierLine(setUpForthPickupPose, forthPickupPose))
                .setLinearHeadingInterpolation(setUpForthPickupPose.getHeading(), forthPickupPose.getHeading())
                .build();

        scoreFourthPickup = follower.pathBuilder()
                .addPath(new BezierCurve(forthPickupPose, returnThirdControlPoint, scorePose))
                .setLinearHeadingInterpolation(forthPickupPose.getHeading(), scorePose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.openGate();
                intake.intakeOff();
                shooter.enableFlywheels();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            // === PRELOAD FIRE ===
            case 1:
                handleFiring(2, grabFirstSpikemark);
                break;

            case 2:
                handleGateClose(3);
                break;

            case 3:
                handleIntakeDelay(4);
                break;

            // === FIRST SPIKEMARK ===
            case 4:
                handlePickupWait(5, scoreFirstSpikemark, intakeTimer, firstSpikeWaitTime);
                break;

            case 5:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(6);
                }
                break;

            // === SECOND SPIKEMARK FIRE ===
            case 6:
                handleFiring(7, grabSecondSpikemark);
                break;

            case 7:
                handleGateClose(8);
                break;

            case 8:
                handleIntakeDelay(9);
                break;

            // === SECOND SPIKEMARK → GATE ===
            case 9:
                handlePickupWait(10, secondGateOpen, intakeTimer, intakeTime);
                break;

            case 10:
                handlePickupWait(11, scoreSecondSpikemark, gateTimer, gateWaitTime);
                break;

            case 11:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(12);
                }
                break;

            // === THIRD SPIKEMARK FIRE ===
            case 12:
                handleFiring(13, grabThirdSpikemark);
                break;

            case 13:
                handleGateClose(14);
                break;

            case 14:
                handleIntakeDelay(15);
                break;

            // === THIRD SPIKEMARK ===
            case 15:
                handlePickupWait(16, scoreThirdSpikemark, intakeTimer, intakeTime);
                break;

            case 16:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(17);
                }
                break;

            // === FOURTH PICKUP FIRE ===
            case 17:
                handleFiring(18, grabFourthPickup);
                break;

            case 18:
                handleGateClose(19);
                break;

            case 19:
                handleIntakeDelay(20);
                break;

            // === FOURTH PICKUP ===
            case 20:
                handlePickupWaitTimed(21, scoreFourthPickup, fourthPickupWaitTime);
                break;

            case 21:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(22);
                }
                break;

            // === FINAL FIRE + PARK ===
            case 22:
                handleFiring(999, Park);
                break;

            case 999:
                if (!follower.isBusy()) {
                    PoseStorage.currentPose = follower.getPose();
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; pathTimer.reset(); }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        firingTimer = new ElapsedTime();
        shooterGateTimer = new ElapsedTime();
        gateTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        leds = new LEDSubsystem(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        if (follower.getPose() != null) {
            PoseStorage.currentPose = follower.getPose();
            shooter.alignTurret(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading(),
                    false,
                    telemetry,
                    follower.getVelocity().getMagnitude(),
                    follower.getVelocity().getTheta(),
                    -14,
                    true
            );
        }
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    private void handleFiring(int nextState, PathChain nextPath) {
        if (!follower.isBusy() && shooter.isAtSpeed() && !readyToFire) {
            firingTimer.reset();
            readyToFire = true;
        }

        if (readyToFire) {
            shooter.isFiring = true;
            intake.intakeCustom();
        }

        if (readyToFire && firingTimer.seconds() > shootTime) {
            shooter.isFiring = false;
            shooterGateTimer.reset();
            intake.intakeOff();
            follower.followPath(nextPath, true);
            readyToFire = false;
            setPathState(nextState);
        }
    }

    private void handleGateClose(int nextState) {
        if (shooterGateTimer.seconds() > shooterGateOpenTime) {
            shooter.closeGate();
            shooterGateTimer.reset();
            setPathState(nextState);
        }
    }

    private void handleIntakeDelay(int nextState) {
        if (shooterGateTimer.seconds() > shooterGateOpenTime) {
            intake.intakeFull();
            setPathState(nextState);
        }
    }

    private void handlePickupWait(int nextState, PathChain nextPath, ElapsedTime timer, double waitTime) {
        if (!follower.isBusy() && !arrivedAtPickup) {
            timer.reset();
            arrivedAtPickup = true;
        }

        if (arrivedAtPickup && timer.seconds() > waitTime) {
            intake.intakeOff();
            shooterGateTimer.reset();
            follower.followPath(nextPath, true);
            arrivedAtPickup = false;
            setPathState(nextState);
        }
    }

    private void handlePickupWaitTimed(int nextState, PathChain nextPath, double waitTime) {
        if (pathTimer.seconds() > waitTime) {
            intake.intakeOff();
            shooterGateTimer.reset();
            follower.setMaxPower(1.0);
            follower.followPath(nextPath, true);
            setPathState(nextState);
        }
    }
}