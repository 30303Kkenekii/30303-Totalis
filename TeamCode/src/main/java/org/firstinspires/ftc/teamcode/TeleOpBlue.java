package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp Blue - FINAL MASTER", group = "Main")
public class TeleOpBlue extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    enum MainState { MANUAL, AUTO_DRIVING, AUTO_FIRE_SEQUENCE }
    private MainState mainState = MainState.MANUAL;

    enum IntakeState { IDLE, INTAKING, STALLED, EJECTING }
    private IntakeState intakeState = IntakeState.IDLE;

    private Pose safePose = new Pose(8, 8, 0);
    private boolean lastX = false, lastY = false, lastA = false, lastLB = false, lastRB = false;
    private double driverTrim = 0;
    private ElapsedTime firingTimer = new ElapsedTime();
    private boolean firingActive = false, autoFireSignal = false;

    public static double SHOOT_1_X = -35, SHOOT_1_Y = 20, SHOOT_1_H = 130;
    public static double SHOOT_2_X = -45, SHOOT_2_Y = -10, SHOOT_2_H = 180;
    public static double PARK_X = 48, PARK_Y = 48, PARK_H = 90;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        leds = new LEDSubsystem(hardwareMap);

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

        follower.setStartingPose(new Pose(8, 8, 0));
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose(); Vector vv = follower.getVelocity();
            if (cp != null) safePose = cp;

            // Trim
            if (gamepad1.left_bumper && !lastLB) driverTrim += 1.0;
            if (gamepad1.right_bumper && !lastRB) driverTrim -= 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            // Updates
            if (vv != null) shooter.alignTurret(safePose.getX(), safePose.getY(), safePose.getHeading(), false, telemetry, vv.getMagnitude(), vv.getTheta(), driverTrim, false);
            leds.updateRPMIndicator(shooter.getCurrentVelocity(), shooter.getTargetVelocity());

            if (shooter.isAtSpeed() && !firingActive) gamepad1.rumble(0.4, 0.4, 50);

            handleDrive();
            handleIntake();
            handleFiring();
            handleNav();

            if (mainState == MainState.AUTO_DRIVING && !follower.isBusy()) {
                if (Math.hypot(safePose.getX()-PARK_X, safePose.getY()-PARK_Y) > 10) { mainState = MainState.AUTO_FIRE_SEQUENCE; autoFireSignal = true; }
                else { mainState = MainState.MANUAL; }
            } else if (mainState == MainState.AUTO_FIRE_SEQUENCE && !autoFireSignal && !firingActive) { mainState = MainState.MANUAL; }

            telemetry.addData("Flywheels", shooter.areFlywheelsEnabled() ? "ENABLED" : "DISABLED");
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
        if (gamepad1.b || firingActive) intakeState = IntakeState.IDLE;

        switch (intakeState) {
            case IDLE: if (gamepad1.left_trigger > 0.1) intake.intakeReverse(); else if (!firingActive) intake.intakeOff(); break;
            case INTAKING: intake.intakeFull(); if (intake.isStalled()) intakeState = IntakeState.STALLED; break;
            case EJECTING: intake.intakeReverse(); break;
            case STALLED: intake.intakeOff(); gamepad1.rumble(50); break;
        }
    }

    private void handleFiring() {
        boolean trigger = (gamepad1.a && !lastA) || (gamepad1.right_trigger > 0.5) || autoFireSignal;
        if (trigger && !firingActive && shooter.isAtSpeed()) { firingActive = true; firingTimer.reset(); autoFireSignal = false; }
        lastA = gamepad1.a;
        if (firingActive) {
            shooter.isFiring = true;
            intake.intakeOff();
            if (firingTimer.seconds() > 0.1) shooter.openGate();
            if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
            if (firingTimer.seconds() > 1.2) { firingActive = false; shooter.isFiring = false; shooter.closeGate(); intake.intakeOff(); }
        }
    }

    private void handleNav() {
        if (gamepad1.start) startNav(SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H);
        if (gamepad1.share || gamepad1.options) startNav(SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H);
        if (gamepad1.right_stick_button) startNav(PARK_X, PARK_Y, PARK_H);

        // Corrected Toggle Logic: These functions now own the state
        if (gamepad1.dpad_left) shooter.disableFlywheels();
        if (gamepad1.dpad_right) shooter.enableFlywheels();
    }

    private void startNav(double x, double y, double h) {
        mainState = MainState.AUTO_DRIVING;
        follower.followPath(follower.pathBuilder().addPath(new BezierLine(safePose, new Pose(x,y,Math.toRadians(h))))
                .setLinearHeadingInterpolation(safePose.getHeading(), Math.toRadians(h)).build(), true);
    }
}