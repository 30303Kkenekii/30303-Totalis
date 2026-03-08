package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "TeleOp Red - FINAL MASTER", group = "Main")
public class TeleOpRed extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private LEDSubsystem leds;
    private LiftSubsystem lift;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    enum IntakeState { IDLE, INTAKING, STALLED, EJECTING }
    private IntakeState intakeState = IntakeState.IDLE;

    private Pose safePose = new Pose(0, 0, 0);
    private boolean lastX = false, lastY = false, lastA = false, lastLB = false, lastRB = false, lastUp = false, lastDown = false;
    private double driverTrim = -5;
    private ElapsedTime firingTimer = new ElapsedTime();
    private boolean firingActive = false;

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

            if (gamepad1.start) {
                follower.setPose(new Pose(0, 0, 0));
                gamepad1.rumble(500);
            }

            if (gamepad1.dpad_up && !lastUp) ShooterSubsystem.sE += 10;
            if (gamepad1.dpad_down && !lastDown) ShooterSubsystem.sE -= 10;
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;

            if (gamepad1.left_bumper && !lastLB) driverTrim += 1.0;
            if (gamepad1.right_bumper && !lastRB) driverTrim -= 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            if (vv != null) shooter.alignTurret(safePose.getX(), safePose.getY(), safePose.getHeading(), false, telemetry, vv.getMagnitude(), vv.getTheta(), driverTrim, false);
            leds.updateRPMIndicator(shooter.getCurrentVelocity(), shooter.getTargetVelocity());

            if (shooter.isAtSpeed() && !firingActive) gamepad1.rumble(0.5, 0.5, 50);

            handleDrive();
            handleIntake();
            handleFiring();

            if (gamepad1.dpad_left) shooter.disableFlywheels();
            if (gamepad1.dpad_right) shooter.enableFlywheels();
            telemetry.update();
        }
    }

    private void handleDrive() {
        double y = -gamepad1.right_stick_y, x = gamepad1.right_stick_x * 1.1, rx = gamepad1.left_stick_x;
        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / den); leftBack.setPower((y - x + rx) / den);
        rightFront.setPower((y - x - rx) / den); rightBack.setPower((y + x - rx) / den);
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
            case STALLED: intake.intakeOff(); break;
        }
    }

    private void handleFiring() {
        boolean trigger = (gamepad1.a && !lastA) || (gamepad1.right_trigger > 0.5);
        if (trigger && !firingActive && shooter.isAtSpeed()) { firingActive = true; firingTimer.reset(); }
        lastA = gamepad1.a;

        if (firingActive) {
            shooter.isFiring = true; intake.intakeOff();
            if (firingTimer.seconds() > 0.1) shooter.openGate();
            if (firingTimer.seconds() > (0.1 + ShooterSubsystem.gateToFeedDelay)) intake.intakeCustom();
            if (firingTimer.seconds() > 1.2) {
                firingActive = false; // REVERTED: Just turns off
                shooter.closeGate(); shooter.isFiring = false;
                intake.intakeOff();
            }
        }
    }
}