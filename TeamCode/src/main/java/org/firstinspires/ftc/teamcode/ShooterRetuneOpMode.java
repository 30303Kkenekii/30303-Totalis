package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "!!! SHOOTER RETUNE & MATH TEST !!!", group = "Calibration")
public class ShooterRetuneOpMode extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    private boolean isArmed = false;
    private boolean isTestingMath = false; // Toggle with Y
    private double driverTrim = 0;
    private boolean lastLB = false, lastRB = false, lastY = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(new Pose(8, 8, 0));

        while (opModeInInit()) {
            telemetry.addLine("RETUNE MODE READY");
            telemetry.addLine("X: ARM | B: DISARM | Y: TOGGLE MATH TEST");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose();

            // RAW DRIVE
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;
            double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / den);
            leftBack.setPower((y - x + rx) / den);
            rightFront.setPower((y - x - rx) / den);
            rightBack.setPower((y + x - rx) / den);

            if (gamepad1.x) isArmed = true;
            if (gamepad1.b) isArmed = false;

            // Toggle between Manual Sliders and your New Regression Math
            if (gamepad1.y && !lastY) isTestingMath = !isTestingMath;
            lastY = gamepad1.y;

            if (gamepad1.left_bumper && !lastLB) driverTrim -= 1.0;
            if (gamepad1.right_bumper && !lastRB) driverTrim += 1.0;
            lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;

            double dist = shooter.distToGoal(cp.getX(), cp.getY(), false);

            if (isArmed) {
                shooter.openGate();
                if (isTestingMath) {
                    // TEST MODE: Uses the y= equations you just provided
                    shooter.setFlywheelVelocity(shooter.getRegressionRPM(dist));
                    shooter.setHoodPosition(shooter.getRegressionHood(dist));
                } else {
                    // MANUAL MODE: Uses Dashboard Sliders
                    shooter.setFlywheelVelocity(ShooterSubsystem.tuningRPM);
                    shooter.setHoodPosition(ShooterSubsystem.tuningHoodPos);
                }
                shooter.alignTurret(cp.getX(), cp.getY(), cp.getHeading(), false, telemetry, 0, 0, driverTrim);
                intake.intakeFull();
            } else {
                shooter.closeGate();
                shooter.setFlywheelVelocity(0);
                intake.intakeOff();
                shooter.setTurretPosition(ShooterSubsystem.tuningTurretTicks);
            }

            telemetry.addData("MODE", isTestingMath ? "TESTING REGRESSION MATH" : "MANUAL SLIDERS");
            telemetry.addData("DISTANCE", dist);
            telemetry.addData("TARGET RPM", isTestingMath ? shooter.getRegressionRPM(dist) : ShooterSubsystem.tuningRPM);
            telemetry.addData("TARGET HOOD", isTestingMath ? shooter.getRegressionHood(dist) : ShooterSubsystem.tuningHoodPos);
            telemetry.update();
        }
    }
}