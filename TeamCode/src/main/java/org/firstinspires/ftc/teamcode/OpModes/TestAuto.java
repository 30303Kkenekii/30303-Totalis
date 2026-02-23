package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TestAuto_3Sec_Hold", group = "Autonomous")
public class TestAuto extends OpMode {

    private Follower follower;
    private int pathState;
    private ElapsedTime pathTimer;

    // --- COORDINATES ---
    private final Pose startPose = new Pose(22.5, 118.5, Math.toRadians(140.5));
    private final Pose scorePose = new Pose(58, 85.5, Math.toRadians(144));
    private final Pose firstPickupPose = new Pose(15, 58.2, Math.toRadians(180));
    private final Pose gatePose = new Pose(13, 57.4, Math.toRadians(147 ));
    private final Pose lastPickupPose = new Pose(20, 85.5, Math.toRadians(180));
    private final Pose lastScorePose = new Pose(50, 115, Math.toRadians(151));

    private final Pose controlPoint = new Pose(59, 58);

    // --- PATHS ---
    private PathChain scorePreload, grabSecondSpikemark, scoreSecondSpikemark,
            gateOpen1, score1, gateOpen2, score2, gateOpen3, score3,
            gateOpen4, score4, grabFirstSpikemark, scoreFirstSpikemark;

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

        gateOpen1 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint, gatePose)).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading()).build();
        score1 = follower.pathBuilder().addPath(new BezierCurve(gatePose, controlPoint, scorePose)).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading()).build();

        gateOpen2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint, gatePose)).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading()).build();
        score2 = follower.pathBuilder().addPath(new BezierCurve(gatePose, controlPoint, scorePose)).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading()).build();

        gateOpen3 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint, gatePose)).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading()).build();
        score3 = follower.pathBuilder().addPath(new BezierCurve(gatePose, controlPoint, scorePose)).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading()).build();

        gateOpen4 = follower.pathBuilder().addPath(new BezierCurve(scorePose, controlPoint, gatePose)).setLinearHeadingInterpolation(scorePose.getHeading(), gatePose.getHeading()).build();
        score4 = follower.pathBuilder().addPath(new BezierCurve(gatePose, controlPoint, scorePose)).setLinearHeadingInterpolation(gatePose.getHeading(), scorePose.getHeading()).build();

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
        // We only move to the next state if the robot is NOT busy (settled) AND 3 seconds have passed
        boolean finishedSwitch = !follower.isBusy() && pathTimer.seconds() > 3.0;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(grabSecondSpikemark, true);
                    setPathState(2);
                }
                break;

            case 2: // Hold at firstPickupPose
                if (finishedSwitch) {
                    follower.followPath(scoreSecondSpikemark, true);
                    setPathState(3);
                }
                break;

            case 3: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(gateOpen1, true);
                    setPathState(4);
                }
                break;

            case 4: // Hold at gatePose (1)
                if (finishedSwitch) {
                    follower.followPath(score1, true);
                    setPathState(5);
                }
                break;

            case 5: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(gateOpen2, true);
                    setPathState(6);
                }
                break;

            case 6: // Hold at gatePose (2)
                if (finishedSwitch) {
                    follower.followPath(score2, true);
                    setPathState(7);
                }
                break;

            case 7: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(gateOpen3, true);
                    setPathState(8);
                }
                break;

            case 8: // Hold at gatePose (3)
                if (finishedSwitch) {
                    follower.followPath(score3, true);
                    setPathState(9);
                }
                break;

            case 9: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(gateOpen4, true);
                    setPathState(10);
                }
                break;

            case 10: // Hold at gatePose (4)
                if (finishedSwitch) {
                    follower.followPath(score4, true);
                    setPathState(11);
                }
                break;

            case 11: // Hold at scorePose
                if (finishedSwitch) {
                    follower.followPath(grabFirstSpikemark, true);
                    setPathState(12);
                }
                break;

            case 12: // Hold at lastPickupPose
                if (finishedSwitch) {
                    follower.followPath(scoreFirstSpikemark, true);
                    setPathState(13);
                }
                break;

            case 13: // Hold at lastScorePose
                if (finishedSwitch) {
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Time in State", String.format("%.2f", pathTimer.seconds()));
        telemetry.addData("Is Settled", !follower.isBusy());
        telemetry.addLine("\n--- CURRENT POSE ---");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}