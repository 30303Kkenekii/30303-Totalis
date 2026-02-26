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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
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
    private final Pose scorePose = new Pose(55.8, 81, Math.toRadians(141));
    private final Pose firstSpikeMarkPose = new Pose(19.5068, 81, Math.toRadians(180));
    private final Pose secondSpikeMarkPose = new Pose(11.3953, 53, Math.toRadians(180));
    private final Pose thirdSpikeMarkPose = new Pose(11.3953, 29, Math.toRadians(180));

    private final Pose gatePose = new Pose(14, 57, Math.toRadians(149.794));

    private final Pose lastScorePose = new Pose(56.2739, 109.3317, Math.toRadians(155.7515));
    private final Pose secondControlPoint = new Pose(65, 50);
    private final Pose thirdControlPoint = new Pose(65, 26);

    public double shootTime = 0.6;

    private PathChain scorePreload, grabFirstSpikemark, scoreFirstSpikemark, grabSecondSpikemark, scoreSecondSpikemark, grabThirdSpikemark, scoreThirdSpikemark;

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
                .addPath(new BezierCurve(secondSpikeMarkPose, secondControlPoint, scorePose))
                .setLinearHeadingInterpolation(secondSpikeMarkPose.getHeading(), scorePose.getHeading())
                .build();

        grabThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, thirdControlPoint, thirdSpikeMarkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdSpikeMarkPose.getHeading())
                .build();

        scoreThirdSpikemark = follower.pathBuilder()
                .addPath(new BezierCurve(thirdSpikeMarkPose, thirdControlPoint, lastScorePose))
                .setLinearHeadingInterpolation(thirdSpikeMarkPose.getHeading(), lastScorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.intakeOff();
                shooter.openGate();
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
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabFirstSpikemark, true);
                    setPathState(3);
                }
                break;

            case 3:
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    follower.followPath(scoreFirstSpikemark, true);
                    setPathState(4); }
                break;

            case 4:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(5); firingTimer.reset(); }
                break;

            case 5: // CYCLE 1 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabSecondSpikemark, true);
                    setPathState(6);
                }
                break;

            case 6:
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    follower.followPath(scoreSecondSpikemark, true);
                    setPathState(7); }
                break;

            case 7:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(8); firingTimer.reset(); }
                break;

            case 8: // CYCLE 2 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabThirdSpikemark, true);
                    setPathState(9);
                }
                break;

            case 9:
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    follower.followPath(scoreThirdSpikemark, true);
                    setPathState(10); }
                break;

            case 10:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(11); firingTimer.reset(); }
                break;

            case 11: // CYCLE 3 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    setPathState(12);
                }
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; pathTimer.reset(); }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        firingTimer = new ElapsedTime();
        gateTimer = new ElapsedTime();
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