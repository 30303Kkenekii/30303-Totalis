package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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
@Autonomous(name = "🟦🟦🟦BlueFarAuto🟦🟦🟦", group = "Autonomous")
public class BlueFarSide extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime shooterGateTimer;
    private ElapsedTime wallPickupTimer;

    // --- COORDINATES ---
    private final Pose startPose = new Pose(56, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(55, 14, Math.toRadians(115));
    private final Pose endPose = new Pose(35, 12, Math.toRadians(180));
    private final Pose wallPose = new Pose(12, 10, Math.toRadians(180));

    public static double shootTime = .9;
    public static double shooterGateOpenTime = .3;
    public static double wallPickupTime = 1.0;

    private boolean readyToFire = false;
    private boolean arrivedAtPickup = false;

    private PathChain scorePreload, driveToWall, driveToScore, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        driveToWall = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, wallPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), wallPose.getHeading())
                .build();

        driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(wallPose, scorePose))
                .setLinearHeadingInterpolation(wallPose.getHeading(), scorePose.getHeading())
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
                handleFiring(2, driveToWall);
                break;

            case 2:
                handleGateClose(3);
                break;

            case 3:
                handleIntakeDelay(4);
                break;

            // === WALL PICKUP 1 ===
            case 4:
                handlePickupWait(5, driveToScore, wallPickupTimer, wallPickupTime);
                break;

            case 5:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(6);
                }
                break;

            // === CYCLE 1 FIRE ===
            case 6:
                handleFiring(7, driveToWall);
                break;

            case 7:
                handleGateClose(8);
                break;

            case 8:
                handleIntakeDelay(9);
                break;

            // === WALL PICKUP 2 ===
            case 9:
                handlePickupWait(10, driveToScore, wallPickupTimer, wallPickupTime);
                break;

            case 10:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(11);
                }
                break;

            // === CYCLE 2 FIRE ===
            case 11:
                handleFiring(12, driveToWall);
                break;

            case 12:
                handleGateClose(13);
                break;

            case 13:
                handleIntakeDelay(14);
                break;

            // === WALL PICKUP 3 ===
            case 14:
                handlePickupWait(15, driveToScore, wallPickupTimer, wallPickupTime);
                break;

            case 15:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(16);
                }
                break;

            // === CYCLE 3 FIRE ===
            case 16:
                handleFiring(17, driveToWall);
                break;

            case 17:
                handleGateClose(18);
                break;

            case 18:
                handleIntakeDelay(19);
                break;

            // === WALL PICKUP 4 ===
            case 19:
                handlePickupWait(20, driveToScore, wallPickupTimer, wallPickupTime);
                break;

            case 20:
                if (shooterGateTimer.seconds() > shooterGateOpenTime) {
                    shooter.openGate();
                    setPathState(21);
                }
                break;

            // === CYCLE 4 FIRE ===
            case 21:
                handleFiring(22, driveToWall);
                break;

            case 22:
                handleGateClose(23);
                break;

            case 23:
                handleIntakeDelay(24);
                break;

            // === WALL PICKUP 5 ===
            case 24:
                handlePickupWait(25, driveToScore, wallPickupTimer, wallPickupTime);
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
        wallPickupTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        leds = new LEDSubsystem(hardwareMap);
        follower.setStartingPose(startPose);
        ShooterSubsystem.isFar = true;
        IntakeSubsystem.ASlowDown = .5;
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
                    -4,
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