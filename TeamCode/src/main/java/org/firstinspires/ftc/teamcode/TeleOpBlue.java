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
@TeleOp(name = "TeleOp Blue - FINAL", group = "Main")
public class TeleOpBlue extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    enum MainState { MANUAL, AUTO_DRIVING, AUTO_FIRE_SEQUENCE }
    private MainState currentMainState = MainState.MANUAL;

    enum IntakeState { IDLE, INTAKING, STALLED }
    private IntakeState currentIntakeState = IntakeState.IDLE;

    private Pose safePose = new Pose(8, 8, 0);
    private boolean flywheelsPowered = true;
    private boolean lastX = false, lastA = false, lastLB = false, lastRB = false;
    private double driverTrim = 0;

    private ElapsedTime firingTimer = new ElapsedTime();
    private boolean firingActive = false;
    private boolean autoFireSignal = false;

    // --- BLUE POSITIONS (Mirrored Y) ---
    public static double B_SHOOT_1_X = -35, B_SHOOT_1_Y = -20, B_SHOOT_1_H = -130;
    public static double B_SHOOT_2_X = -45, B_SHOOT_2_Y = 10, B_SHOOT_2_H = -180;
    public static double B_PARK_X = 48, B_PARK_Y = -48, B_PARK_H = 90;

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

            if (gamepad1.left_bumper && !lastLB) driverTrim += 1.0;
            if (gamepad1.right_bumper && !lastRB) driverTrim -= 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            ShooterSubsystem.flywheelsEnabled = flywheelsPowered;
            if (currentVel != null) {
                // PARAMETER SET TO TRUE FOR BLUE GOAL
                shooter.alignTurret(safePose.getX(), safePose.getY(), safePose.getHeading(), true, telemetry, currentVel.getMagnitude(), currentVel.getTheta(), driverTrim);
            }
            leds.updateRPMIndicator(shooter.getCurrentVelocity(), shooter.getTargetVelocity());

            handleDrive();
            handleIntakeLogic();
            handleFiringSequence();
            handleNavigation();

            if (currentMainState == MainState.AUTO_DRIVING && !follower.isBusy()) {
                double distToPark = Math.hypot(cp.getX() - B_PARK_X, cp.getY() - B_PARK_Y);
                if (distToPark > 10) {
                    currentMainState = MainState.AUTO_FIRE_SEQUENCE;
                    autoFireSignal = true;
                } else {
                    currentMainState = MainState.MANUAL;
                }
            } else if (currentMainState == MainState.AUTO_FIRE_SEQUENCE && !autoFireSignal && !firingActive) {
                currentMainState = MainState.MANUAL;
            }

            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Intake", currentIntakeState);
            telemetry.update();
        }
    }

    private void handleDrive() {
        if (currentMainState == MainState.MANUAL) {
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;
            double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / den);
            leftBack.setPower((y - x + rx) / den);
            rightFront.setPower((y - x - rx) / denominator(y,x,rx));
            rightBack.setPower((y + x - rx) / denominator(y,x,rx));
        }
    }

    private double denominator(double y, double x, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }

    private void handleIntakeLogic() {
        if (gamepad1.x && !lastX) {
            currentIntakeState = (currentIntakeState == IntakeState.INTAKING) ? IntakeState.IDLE : IntakeState.INTAKING;
        }
        lastX = gamepad1.x;

        if (gamepad1.b) {
            currentIntakeState = IntakeState.IDLE;
            if (currentMainState != MainState.MANUAL) cancelAuto();
        }

        switch (currentIntakeState) {
            case IDLE:
                if (gamepad1.left_trigger > 0.1) intake.intakeReverse();
                else if (!firingActive) intake.intakeOff();
                break;
            case INTAKING:
                intake.intakeFull();
                if (intake.isStalled()) currentIntakeState = IntakeState.STALLED;
                break;
            case STALLED:
                intake.intakeOff();
                gamepad1.rumble(0.5, 0.5, 50);
                if (firingActive) currentIntakeState = IntakeState.IDLE;
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
        if (gamepad1.start) startNav(B_SHOOT_1_X, B_SHOOT_1_Y, B_SHOOT_1_H);
        if (gamepad1.share || gamepad1.options) startNav(B_SHOOT_2_X, B_SHOOT_2_Y, B_SHOOT_2_H);
        if (gamepad1.right_stick_button) startNav(B_PARK_X, B_PARK_Y, B_PARK_H);

        if (gamepad1.dpad_left) flywheelsPowered = false;
        if (gamepad1.dpad_right) flywheelsPowered = true;
    }

    private void startNav(double x, double y, double h) {
        currentMainState = MainState.AUTO_DRIVING;
        Pose target = new Pose(x, y, Math.toRadians(h));
        follower.followPath(follower.pathBuilder().addPath(new BezierLine(safePose, target))
                .setLinearHeadingInterpolation(safePose.getHeading(), target.getHeading()).build(), true);
    }

    private void cancelAuto() {
        currentMainState = MainState.MANUAL;
        firingActive = false;
        follower.breakFollowing();
        shooter.closeGate();
    }
}