package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp Red - FINAL", group = "Main")
public class TeleOpRed extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private LiftSubsystem lift;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    enum MainState { MANUAL, AUTO_DRIVING, AUTO_FIRE_SEQUENCE }
    private MainState mainState = MainState.MANUAL;

    enum IntakeState { IDLE, INTAKING, STALLED, EJECTING }
    private IntakeState intakeState = IntakeState.IDLE;

    private Pose safePose = new Pose(0, 0, 0);
    private boolean lastX = false, lastY = false, lastA = false, lastLB = false, lastRB = false, lastUp = false, lastDown = false;
    private double driverTrim = -5;
    private ElapsedTime firingTimer = new ElapsedTime();
    private boolean firingActive = false;

    // Red Coordinates
    public static double SHOOT_1_X = -35, SHOOT_1_Y = 20, SHOOT_1_H = 130;
    public static double SHOOT_2_X = -45, SHOOT_2_Y = -10, SHOOT_2_H = 180;
    public static double PARK_X = 8, PARK_Y = 8, PARK_H = 0;

    private Pose targetPose;
    public static double ARRIVAL_TOLERANCE = 1.5;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        leds = new LEDSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        follower.setStartingPose(PoseStorage.currentPose);
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose(); Vector vv = follower.getVelocity();
            if (cp != null) safePose = cp;

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) lift.liftUp();
            else lift.stop();

            if (gamepad1.start) { follower.setPose(new Pose(0, 0, 0)); gamepad1.rumble(500); }

            if (gamepad1.dpad_up && !lastUp) ShooterSubsystem.sIntercept += 10;
            if (gamepad1.dpad_down && !lastDown) ShooterSubsystem.sIntercept -= 10;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

            if (gamepad1.left_bumper && !lastLB) driverTrim += 1.0;
            if (gamepad1.right_bumper && !lastRB) driverTrim -= 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            if (vv != null) shooter.alignTurret(safePose.getX(), safePose.getY(), safePose.getHeading(), false, telemetry, vv.getMagnitude(), vv.getTheta(), driverTrim, false);
            leds.updateRPMIndicator(shooter.getCurrentVelocity(), shooter.getTargetVelocity());

            // SPEED VIBRATION
            if (shooter.isAtSpeed() && !firingActive) gamepad1.rumble(0.4, 0.4, 50);

            handleDrive();
            handleIntake();
            handleFiring();
            handleNavButtons();

            // Auto-Drive Logic
            if (mainState == MainState.AUTO_DRIVING) {
                double dist = Math.hypot(targetPose.getX() - safePose.getX(), targetPose.getY() - safePose.getY());
                if (Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) cancelAuto();
                else if (dist < ARRIVAL_TOLERANCE || !follower.isBusy()) {
                    follower.holdPoint(targetPose); // Stop the roll
                    mainState = MainState.AUTO_FIRE_SEQUENCE;
                }
            } else if (mainState == MainState.AUTO_FIRE_SEQUENCE && !firingActive) {
                cancelAuto();
            }
            telemetry.update();
        }
    }

    private void handleDrive() {
        if (mainState == MainState.MANUAL) {
            double y = -gamepad1.right_stick_y, x = gamepad1.right_stick_x * 1.1, rx = gamepad1.left_stick_x;
            double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / den); leftBack.setPower((y - x + rx) / den);
            rightFront.setPower((y - x - rx) / den); rightBack.setPower((y + x - rx) / den);
        }
    }

    private void handleIntake() {
        if (gamepad1.x && !lastX) intakeState = (intakeState == IntakeState.INTAKING) ? IntakeState.IDLE : IntakeState.INTAKING;
        if (gamepad1.y && !lastY) intakeState = (intakeState == IntakeState.EJECTING) ? IntakeState.IDLE : IntakeState.EJECTING;
        lastX = gamepad1.x; lastY = gamepad1.y;
        if (gamepad1.b) { intakeState = IntakeState.IDLE; if (mainState != MainState.MANUAL) cancelAuto(); }

        switch (intakeState) {
            case IDLE: if (gamepad1.left_trigger > 0.1) intake.intakeReverse(); else if (!firingActive) intake.intakeOff(); break;
            case INTAKING: intake.intakeFull(); if (intake.isStalled()) intakeState = IntakeState.STALLED; break;
            case EJECTING: intake.intakeReverse(); break;
            case STALLED: intake.intakeOff(); break;
        }
    }

    private void handleFiring() {
        boolean trigger = (gamepad1.a && !lastA) || (gamepad1.right_trigger > 0.5) || (mainState == MainState.AUTO_FIRE_SEQUENCE);
        if (trigger && !firingActive && shooter.isAtSpeed()) { firingActive = true; firingTimer.reset(); }
        lastA = gamepad1.a;

        if (firingActive) {
            if (firingTimer.seconds() < 0.1) intake.intakeOff();
            if (firingTimer.seconds() > 0.1) shooter.openGate();
            if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
            if (firingTimer.seconds() > 1.2) {
                shooter.closeGate();
                firingActive = false;
                intakeState = IntakeState.INTAKING; // RESUME AUTOMATICALLY
            }
        }
    }

    private void handleNavButtons() {
        if (gamepad1.share) startNav(SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H);
        if (gamepad1.dpad_left) shooter.disableFlywheels();
        if (gamepad1.dpad_right) shooter.enableFlywheels();
    }

    private void startNav(double x, double y, double h) {
        mainState = MainState.AUTO_DRIVING;
        targetPose = new Pose(x, y, Math.toRadians(h));
        follower.followPath(follower.pathBuilder().addPath(new BezierLine(safePose, targetPose))
                .setLinearHeadingInterpolation(safePose.getHeading(), targetPose.getHeading()).build(), false);
    }

    private void cancelAuto() {
        mainState = MainState.MANUAL; follower.breakFollowing(); firingActive = false;
        leftFront.setPower(0); leftBack.setPower(0); rightFront.setPower(0); rightBack.setPower(0);
    }
}