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
@Autonomous(name = "🟦🟦🟦Blue18Auto🟦🟦🟦", group = "Autonomous")
public class Blue18Ball extends OpMode {

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
    private final Pose startPose = new Pose(23.1422, 118.595, Math.toRadians(140.1586));
    private final Pose scorePose = new Pose(57.3, 84.3, Math.toRadians(141));
    private final Pose firstSpikeMarkPose = new Pose(19.5745, 72.3472, Math.toRadians(180));
    private final Pose secondSpikeMarkPose = new Pose(12, 57.5, Math.toRadians(180));
    private final Pose thirdSpikeMarkPose = new Pose(12, 36, Math.toRadians(180));
    private final Pose setUpForthPickupPose = new Pose(9.5, 53, Math.toRadians(-90));
    private final Pose forthPickupPose = new Pose(10, 10, Math.toRadians(-95));
    private final Pose endPose = new Pose(27, 72, Math.toRadians(180));
    private final Pose gatePose = new Pose(13.8744, 56.8, Math.toRadians(148.4346));
    public Pose secondControlPoint = new Pose(65, 65);
    public Pose returnSecondControlPoint = new Pose(33, 50);
    public Pose thirdControlPoint = new Pose(65, 26);
    public Pose returnThirdControlPoint = new Pose(33, 26);

    public double shootTime = .5;
    public double shooterGateOpenTime = .2;
    public double intakeTime = .2;
    public double firstSpikeWaitTime = 0.8;
    public double gateWaitTime = 0;
    public double fourthPickupWaitTime = 3.7;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;

    private PathChain scorePreload, grabFirstSpikemark, scoreFirstSpikemark,
            grabSecondSpikemark, scoreSecondSpikemark, grabGatePickup, scoreGatePickup,
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

        scoreSecondSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(secondSpikeMarkPose, returnSecondControlPoint, scorePose))
                .setLinearHeadingInterpolation(secondSpikeMarkPose.getHeading(), scorePose.getHeading())
                .build();

        grabGatePickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondControlPoint, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        scoreGatePickup = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, returnSecondControlPoint, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        grabThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, thirdControlPoint, thirdSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdSpikeMarkPose.getHeading())
                .build();

        scoreThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(thirdSpikeMarkPose, returnThirdControlPoint, scorePose))
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

            // === SECOND SPIKEMARK ===
            case 9:
                handlePickupWait(10, scoreSecondSpikemark, intakeTimer, intakeTime);
                break;

            case 10:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(11);
                }
                break;

            // === GATE FIRE ===
            case 11:
                handleFiring(12, grabGatePickup);
                break;

            case 12:
                handleGateClose(13);
                break;

            case 13:
                handleIntakeDelay(14);
                break;

            // === GATE PICKUP ===
            case 14:
                handlePickupWait(15, scoreGatePickup, gateTimer, gateWaitTime);
                break;

            case 15:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(16);
                }
                break;

            // === THIRD SPIKEMARK FIRE ===
            case 16:
                handleFiring(17, grabThirdSpikemark);
                break;

            case 17:
                handleGateClose(18);
                break;

            case 18:
                handleIntakeDelay(19);
                break;

            // === THIRD SPIKEMARK ===
            case 19:
                handlePickupWait(20, scoreThirdSpikemark, intakeTimer, intakeTime);
                break;

            case 20:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(21);
                }
                break;

            // === FOURTH PICKUP FIRE ===
            case 21:
                handleFiring(22, grabFourthPickup);
                break;

            case 22:
                handleGateClose(23);
                break;

            case 23:
                handleIntakeDelay(24);
                break;

            // === FOURTH PICKUP ===
            case 24:
                handlePickupWaitTimed(25, scoreFourthPickup, fourthPickupWaitTime);
                break;

            case 25:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(26);
                }
                break;

            // === FINAL FIRE + PARK ===
            case 26:
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
                    true,
                    telemetry,
                    follower.getVelocity().getMagnitude(),
                    follower.getVelocity().getTheta(),
                    -5,
                    false
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
            follower.followPath(nextPath, true);
            setPathState(nextState);
        }
    }
}