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
    private ElapsedTime gateTimer;
    private ElapsedTime gateOpenTimer;


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
                gateOpenTimer.reset();
                shooter.openGate();
                intake.intakeOff();
                shooter.enableFlywheels();
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && shooter.isAtSpeed()) {
                    firingTimer.reset();
                    setPathState(2);
                }
                break;

            case 2: // PRELOAD FIRE
                shooter.isFiring = true;
                intake.intakeCustom();
                if (firingTimer.seconds() > shootTime) {
                    shooter.isFiring = false;
                    shooter.closeGate();
                    intake.intakeOff();
                    follower.followPath(grabFirstPickup, true);

                    setPathState(3);
                }
                break;

            case 3: // Arrival at First Pickup
                intake.intakeFull();
                if (!follower.isBusy()) {

                    setPathState(4);
                }
                break;

            case 21: // Final Log and Stop
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
        gateOpenTimer = new ElapsedTime();
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
        if (follower.getPose() != null) {
            PoseStorage.currentPose = follower.getPose();
        }
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }
}