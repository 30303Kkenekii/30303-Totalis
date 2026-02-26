package org.firstinspires.ftc.teamcode.OpModes;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueMasterAuto_Final_Fixed", group = "Autonomous")
public class BlueMasterAuto extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime gateTimer;

    private final Pose startPose = new Pose(23.1422, 118.595, Math.toRadians(140.1586));
    private final Pose scorePose = new Pose(55.8, 86.068, Math.toRadians(141));
    private final Pose firstPickupPose = new Pose(11.3953, 53, Math.toRadians(180));
    private final Pose gatePose = new Pose(14, 57, Math.toRadians(149.794));
    private final Pose lastPickupPose = new Pose(19.5068, 80.9698, Math.toRadians(180));
    private final Pose lastScorePose = new Pose(56.2739, 109.3317, Math.toRadians(155.7515));
    private final Pose controlPoint = new Pose(65, 50);

    private PathChain scorePreload, grabSecondSpikemark, scoreSecondSpikemark,
            gateOpen1, score1, gateOpen2, score2, grabFirstSpikemark, scoreFirstSpikemark;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabSecondSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstPickupPose.getHeading())
                .build();

        scoreSecondSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, controlPoint, scorePose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), scorePose.getHeading())
                .build();

        gateOpen1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, controlPoint, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        gateOpen2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint, gatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, controlPoint, scorePose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading())
                .build();

        grabFirstSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lastPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lastPickupPose.getHeading())
                .build();

        scoreFirstSpikemark = follower.pathBuilder()
                .addPath(new BezierLine(lastPickupPose, lastScorePose))
                .setLinearHeadingInterpolation(lastPickupPose.getHeading(), lastScorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.enableFlywheels();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(2); firingTimer.reset();
                }
                break;

            case 2: // PRELOAD FIRE
                intake.intakeOff(); shooter.isFiring = true;
                if (firingTimer.seconds() > 0.1) shooter.openGate();
                if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
                if (firingTimer.seconds() > 1.2) {
                    shooter.isFiring = false; shooter.closeGate(); intake.intakeOff();
                    follower.followPath(grabSecondSpikemark, true);
                    setPathState(3);
                }
                break;

            case 3:
                intake.intakeFull();
                if (!follower.isBusy()) { follower.followPath(scoreSecondSpikemark, true); setPathState(4); }
                break;

            case 4:
                if (!follower.isBusy() && shooter.isAtSpeed()) { setPathState(5); firingTimer.reset(); }
                break;

            case 5: // CYCLE 1 FIRE
                intake.intakeOff(); shooter.isFiring = true;
                if (firingTimer.seconds() > 0.1) shooter.openGate();
                if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
                if (firingTimer.seconds() > 2.0) {
                    shooter.isFiring = false; shooter.closeGate(); intake.intakeOff();
                    follower.followPath(gateOpen1, true);
                    setPathState(6);
                }
                break;

            case 6: // Arrival at Gate 1
                intake.intakeFull();
                if (!follower.isBusy()) { setPathState(7); gateTimer.reset(); }
                break;

            case 7: // Waiting at Gate 1
                if (gateTimer.seconds() > 1.7) { setPathState(8); follower.followPath(score1, true); }
                break;

            case 8:
                if (!follower.isBusy() && shooter.isAtSpeed()) { setPathState(9); firingTimer.reset(); }
                break;

            case 9: // GATE 1 FIRE
                intake.intakeOff(); shooter.isFiring = true;
                if (firingTimer.seconds() > 0.1) shooter.openGate();
                if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
                if (firingTimer.seconds() > 1.2) {
                    shooter.isFiring = false; shooter.closeGate(); intake.intakeOff();
                    follower.followPath(gateOpen2, true);
                    setPathState(10);
                }
                break;

            case 10: // Arrival at Gate 2
                intake.intakeFull();
                if (!follower.isBusy()) { setPathState(11); gateTimer.reset(); }
                break;

            case 11: // Waiting at Gate 2
                if (gateTimer.seconds() > 1.7) { setPathState(12); follower.followPath(score2, true); }
                break;

            case 12:
                if (!follower.isBusy() && shooter.isAtSpeed()) { setPathState(13); firingTimer.reset(); }
                break;

            case 13: // GATE 2 FIRE
                intake.intakeOff(); shooter.isFiring = true;
                if (firingTimer.seconds() > 0.1) shooter.openGate();
                if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
                if (firingTimer.seconds() > 1.2) {
                    shooter.isFiring = false; shooter.closeGate(); intake.intakeOff();
                    // SKIP GATE 3 -> GO TO FIRST SPIKEMARK (ORIGINAL STATE 18)
                    follower.followPath(grabFirstSpikemark, true);
                    setPathState(14);
                }
                break;

            case 14: // PICKUP FIRST SPIKEMARK
                intake.intakeFull();
                if (!follower.isBusy()) { follower.followPath(scoreFirstSpikemark, true); setPathState(15); }
                break;

            case 15:
                if (!follower.isBusy() && shooter.isAtSpeed()) { setPathState(16); firingTimer.reset(); }
                break;

            case 16: // FINAL FIRE
                intake.intakeOff(); shooter.isFiring = true;
                if (firingTimer.seconds() > 0.1) shooter.openGate();
                if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
                if (firingTimer.seconds() > 1.2) {
                    shooter.isFiring = false; shooter.disableFlywheels(); intake.intakeOff();
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; pathTimer.reset(); }

    @Override
    public void init() {
        pathTimer = new ElapsedTime(); firingTimer = new ElapsedTime(); gateTimer = new ElapsedTime();
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

        // RUN ALIGNMENT:
        // - driverOffset = -3.0
        // - isAuto = true (applies autoRPMOffset)
        shooter.alignTurret(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading(),
                true,
                telemetry,
                follower.getVelocity().getMagnitude(),
                follower.getVelocity().getTheta(),
                -4.5,
                true
        );

        telemetry.addData("Path State", pathState);
        telemetry.addData("Auto RPM Offset", ShooterSubsystem.autoRPMOffset);
        telemetry.update();
    }
}