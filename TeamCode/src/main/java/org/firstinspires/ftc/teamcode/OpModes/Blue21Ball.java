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
@Autonomous(name = "🟦🟦🟦Blue21BallAuto🟦🟦🟦", group = "Autonomous")
public class Blue21Ball extends OpMode {

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
    private final Pose secondSpikeMarkPose = new Pose(12, 58, Math.toRadians(180));
    private final Pose thirdSpikeMarkPose = new Pose(12, 36, Math.toRadians(180));
    private final Pose setUpForthPickupPose = new Pose(9.5, 53, Math.toRadians(-90));
    private final Pose forthPickupPose = new Pose(10, 10, Math.toRadians(-95));
    private final Pose endPose = new Pose(27, 72, Math.toRadians(180));
    private final Pose gatePose = new Pose(14, 57, Math.toRadians(149.794));
    private final Pose secondControlPoint = new Pose(65, 55);
    private final Pose returnSecondControlPoint = new Pose(43, 50);
    private final Pose thirdControlPoint = new Pose(65, 26);
    private final Pose returnThirdControlPoint = new Pose(43, 26);

    public double shootTime = 0.6;
    public double shooterGateOpenTime = .3;
    public double intakeTime = .3;
    public double gateWaitTime = 1;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;

    private PathChain scorePreload, grabFirstPickup, scoreFirstPickup, grabGatePickup, scoreGatePickup, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabFirstPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, secondControlPoint, secondSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondSpikeMarkPose.getHeading())
                .build();

        scoreFirstPickup = follower.pathBuilder()
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

            // === CYCLE 1 ===
            case 1: // PRELOAD FIRE
                handleFiring(2, grabFirstPickup);
                break;

            case 2:
                handleGateClose(3);
                break;

            case 3:
                handleIntakeDelay(4);
                break;

            case 4:
                handlePickupWait(5, scoreFirstPickup, intakeTimer, intakeTime);
                break;

            case 5:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(6);
                }
                break;

            // === CYCLE 2 ===
            case 6:
                handleFiring(7, grabGatePickup);
                break;

            case 7:
                handleGateClose(8);
                break;

            case 8:
                handleIntakeDelay(9);
                break;

            case 9:
                handlePickupWait(10, scoreGatePickup, gateTimer, gateWaitTime);
                break;

            case 10:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(11);
                }
                break;

            // === CYCLE 3 ===
            case 11:
                handleFiring(12, grabGatePickup);
                break;

            case 12:
                handleGateClose(13);
                break;

            case 13:
                handleIntakeDelay(14);
                break;

            case 14:
                handlePickupWait(15, scoreGatePickup, gateTimer, gateWaitTime);
                break;

            case 15:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(16);
                }
                break;

            // === CYCLE 4 ===
            case 16:
                handleFiring(17, grabGatePickup);
                break;
            case 17:
                handleGateClose(18);
                break;

            case 18:
                handleIntakeDelay(19);
                break;

            case 19:
                handlePickupWait(20, scoreGatePickup, gateTimer, gateWaitTime);
                break;

            case 20:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(21);
                }
                break;

            // === FINAL FIRE + PARK ===
            case 21:
                handleFiring(22, Park);
                break;

            case 22:
                handleGateClose(23);
                break;

            case 23:
                handleIntakeDelay(999);
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
                    0,
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
}