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
@TeleOp(name = "TeleOp Blue - FINAL MASTER", group = "B")
public class TeleOpBlue extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    enum MainState { MANUAL, AUTO_DRIVING, AUTO_FIRE_SEQUENCE }
    private MainState currentMainState = MainState.MANUAL;

    enum IntakeState { IDLE, INTAKING, EJECTING }
    private IntakeState currentIntakeState = IntakeState.IDLE;

    private Pose safePose = new Pose(8, 8, 0);
    private boolean flywheelsPowered = true;
    private boolean lastX = false, lastY = false, lastA = false;
    private boolean lastLB = false, lastRB = false; // For Trimming
    private double driverMatchOffset = 0; // The nudge value

    private ElapsedTime firingTimer = new ElapsedTime();
    private boolean firingActive = false;
    private boolean autoFireSignal = false;

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
            Pose cp = follower.getPose();
            Vector currentVel = follower.getVelocity();
            if (cp != null) safePose = cp;

            // --- BUMPER TRIMMING ---
            // LB = Aim 1 deg Left | RB = Aim 1 deg Right
            if (gamepad1.left_bumper && !lastLB) driverMatchOffset -= 1.0;
            if (gamepad1.right_bumper && !lastRB) driverMatchOffset += 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            // --- SHOOTER UPDATE ---
            ShooterSubsystem.flywheelsEnabled = flywheelsPowered;
            if (currentVel != null) {
                // Now passing driverMatchOffset
                shooter.alignTurret(safePose.getX(), safePose.getY(), safePose.getHeading(), true, telemetry,
                        currentVel.getMagnitude(), currentVel.getTheta(), driverMatchOffset);
            }
            leds.updateRPMIndicator(shooter.getCurrentVelocity(), shooter.getTargetVelocity());

            handleManualDrive();
            handleIntakeStateMachine();
            handleFiringSequence();
            handleNavigation();

            if (currentMainState == MainState.AUTO_DRIVING && !follower.isBusy()) {
                currentMainState = MainState.AUTO_FIRE_SEQUENCE;
                autoFireSignal = true;
            } else if (currentMainState == MainState.AUTO_FIRE_SEQUENCE && !autoFireSignal && !firingActive) {
                cancelAuto();
            }

            telemetry.addData("Aim Trim (Deg)", driverMatchOffset);
            telemetry.addData("Intake State", currentIntakeState);
            telemetry.update();
        }
    }

    private void handleManualDrive() {
        if (currentMainState == MainState.MANUAL) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);
        }
    }

    private void handleIntakeStateMachine() {
        if (gamepad1.b || firingActive || gamepad1.left_trigger > 0.1) {
            currentIntakeState = IntakeState.IDLE;
        }

        if (gamepad1.x && !lastX) currentIntakeState = IntakeState.INTAKING;
        if (gamepad1.y && !lastY) currentIntakeState = IntakeState.EJECTING;
        lastX = gamepad1.x; lastY = gamepad1.y;

        switch (currentIntakeState) {
            case IDLE:
                if (gamepad1.left_trigger > 0.1) intake.intakeReverse();
                else if (!firingActive) intake.intakeOff();
                break;
            case INTAKING:
                intake.intakeFull();
                break;
            case EJECTING:
                intake.intakeEject();
                break;
        }
    }

    private void handleFiringSequence() {
        boolean triggerPressed = (gamepad1.a && !lastA) || (gamepad1.right_trigger > 0.5);
        if (triggerPressed || autoFireSignal) {
            if (!firingActive) { firingActive = true; firingTimer.reset(); autoFireSignal = false; }
        }
        lastA = gamepad1.a;

        if (firingActive) {
            shooter.openGate();
            if (firingTimer.seconds() > ShooterSubsystem.gateToFeedDelay) intake.intakeCustom();
            if (firingTimer.seconds() > 1.0) {
                firingActive = false;
                shooter.closeGate();
                intake.intakeOff();
            }
        }
    }

    private void handleNavigation() {
        if (gamepad1.start) startNav(SHOOT_1_X, SHOOT_1_Y, SHOOT_1_H);
        if (gamepad1.share || gamepad1.options) startNav(SHOOT_2_X, SHOOT_2_Y, SHOOT_2_H);
        if (gamepad1.right_stick_button) startNav(PARK_X, PARK_Y, PARK_H);

        if (gamepad1.dpad_left) flywheelsPowered = false;
        if (gamepad1.dpad_right) flywheelsPowered = true;
    }

    private void startNav(double x, double y, double h) {
        currentMainState = MainState.AUTO_DRIVING;
        Pose target = new Pose(x, y, Math.toRadians(h));
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(safePose, target))
                .setLinearHeadingInterpolation(safePose.getHeading(), target.getHeading())
                .build(), true);
    }

    private void cancelAuto() {
        currentMainState = MainState.MANUAL;
        firingActive = false;
        follower.breakFollowing();
        shooter.closeGate();
    }
}