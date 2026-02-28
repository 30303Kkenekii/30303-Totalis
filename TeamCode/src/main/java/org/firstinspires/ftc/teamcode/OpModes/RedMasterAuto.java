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
@Autonomous(name = "🟥🟥🟥RedMasterAuto🟥🟥🟥", group = "Autonomous")
public class RedMasterAuto extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private int pathState;

    private ElapsedTime pathTimer;
    private ElapsedTime firingTimer;
    private ElapsedTime gateTimer;
    private ElapsedTime intakeTimer;

    // --- COORDINATES ---
    private final Pose startPose = new Pose(125.3233, 115.5557, Math.toRadians(37.8449));
    private final Pose scorePose = new Pose(90, 84.3, Math.toRadians(29));
    private final Pose firstSpikeMarkPose = new Pose(127.4342, 70, Math.toRadians(0));
    private final Pose secondSpikeMarkPose = new Pose(136, 54, Math.toRadians(-5));
    private final Pose thirdSpikeMarkPose = new Pose(135, 30.3274, Math.toRadians(0));

    private final Pose setUpForthPickupPose = new Pose(136.3332, 54, Math.toRadians(-90));
    private final Pose forthPickupPose = new Pose(135, 5, Math.toRadians(-85));

    private final Pose endPose = new Pose(118, 66, Math.toRadians(0));

    private final Pose gatePose = new Pose(14, 57, Math.toRadians(149.794));

    private final Pose secondControlPoint = new Pose(80, 62);
    private final Pose returnSecondControlPoint = new Pose(110, 55);
    private final Pose thirdControlPoint = new Pose(80, 38);
    private final Pose returnThirdControlPoint = new Pose(110, 36);






    public double shootTime = 0.6;

    private PathChain scorePreload, grabFirstSpikemark, scoreFirstSpikemark, grabSecondSpikemark, scoreSecondSpikemark, grabThirdSpikemark, scoreThirdSpikemark, grabFourthPickup, scoreFourthPickup, Park;

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

            case 3: // Arrival at First Pickup
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    gateTimer.reset(); // YOUR FIX
                    setPathState(4);
                }
                break;

            case 4: // Wait for 0.4s at First Pickup
                if (gateTimer.seconds() > .8) { // YOUR FIX
                    follower.followPath(scoreFirstSpikemark, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(6); firingTimer.reset(); }
                break;

            case 6: // CYCLE 1 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabSecondSpikemark, true);
                    setPathState(7);
                }
                break;

            case 7:
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    intakeTimer.reset();
                    setPathState(8);
                }
                break;
            case 8: // Wait for 0.5s at Second Pickup
                if (intakeTimer.seconds() > .5) { // YOUR FIX
                    follower.followPath(scoreSecondSpikemark, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(10); firingTimer.reset(); }
                break;

            case 10: // CYCLE 2 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabThirdSpikemark, true);
                    setPathState(11);
                }
                break;

            case 11:
                intake.intakeFull();
                if (!follower.isBusy()) {
                    shooter.openGate();
                    intake.intakeOff();
                    intakeTimer.reset();
                    setPathState(12);
                }
                break;
            case 12: // Wait for 0.5s at Third Pickup
                if (intakeTimer.seconds() > .5) { // YOUR FIX
                    follower.followPath(scoreThirdSpikemark, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(14); firingTimer.reset(); }
                break;

            case 14: // CYCLE 3 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabFourthPickup, false);
                    setPathState(15);
                }
                break;

            case 15: // Duration Drive to 4th Pickup
                intake.intakeFull();
                if (pathTimer.seconds() > 4.5) { // YOUR FIX
                    shooter.openGate();
                    intake.intakeOff();
                    follower.followPath(scoreFourthPickup, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    setPathState(17); firingTimer.reset(); }
                break;

            case 17: // CYCLE 4 FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(Park, true);
                    setPathState(18);
                }
                break;

            case 18: // Final Log and Stop
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
        gateTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        leds = new LEDSubsystem(hardwareMap);
        follower.setStartingPose(startPose);
        ShooterSubsystem.sIntercept = 655;
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
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
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }
}