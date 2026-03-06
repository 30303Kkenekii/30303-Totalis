package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Flywheel & Hood Tuning", group = "TeleOp")
public class ShooterRegressionTuner extends OpMode {
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    public static int vel = 0;
    public static double hoodPos = 0.5;

    enum IntakeState { IDLE, INTAKING, STALLED }
    private IntakeState intakeState = IntakeState.IDLE;

    private boolean lastX = false, lastLB = false, lastRB = false;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        shooter.disableFlywheels();
        intake.intakeOff();
    }

    @Override
    public void loop() {
        follower.update();
        Pose cp = follower.getPose();

        // --- 1. DRIVE ---
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;
        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / den); leftBack.setPower((y - x + rx) / den);
        rightFront.setPower((y - x - rx) / den); rightBack.setPower((y + x - rx) / den);

        // --- 2. TUNING INPUTS ---
        if (gamepad1.right_bumper && !lastRB) {
            vel += 100;
            shooter.enableFlywheels(); // FIX: Wake up system
            gamepad1.rumble(100);
        }
        else if (gamepad1.left_bumper && !lastLB) {
            vel -= 100;
            shooter.enableFlywheels(); // FIX: Wake up system
            gamepad1.rumble(100);
        }

        if (gamepad1.dpad_up) hoodPos += 0.001;
        if (gamepad1.dpad_down) hoodPos -= 0.001;
        hoodPos = Range.clip(hoodPos, 0, 0.8);

        // --- 3. INTAKE & FEED LOGIC ---
        if (gamepad1.x && !lastX) intakeState = (intakeState == IntakeState.INTAKING) ? IntakeState.IDLE : IntakeState.INTAKING;
        if (gamepad1.b) intakeState = IntakeState.IDLE;

        switch (intakeState) {
            case IDLE:
                if (gamepad1.a) { shooter.openGate(); intake.intakeCustom(); }
                else {; intake.intakeOff(); }
                break;
            case INTAKING:
                intake.intakeFull();
                if (intake.isStalled()) intakeState = IntakeState.STALLED;
                break;
            case STALLED:
                intake.intakeOff();
                gamepad1.rumble(50);
                if (gamepad1.a) { shooter.openGate(); intake.intakeCustom(); intakeState = IntakeState.IDLE; }
                break;
        }

        lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper; lastX = gamepad1.x;

        // --- 4. HARDWARE ---
        //shooter.setFlywheelVelocity(vel);
        //shooter.setHoodPosition(hoodPos);
        //shooter.setTurretPosition(ShooterSubsystem.tuningTurretTicks);

        // --- 5. TELEMETRY ---
        if (cp != null) {
            double dist = Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(), ShooterSubsystem.blueGoalY - cp.getY());
            telemetry.addData("Dist to Blue Goal", "%.2f", dist);
            telemetry.addData("Target RPM", vel);
            telemetry.addData("Actual RPM", (int)shooter.getCurrentVelocity());
            telemetry.addData("Hood Target", "%.3f", hoodPos);
            telemetry.addData("System Active", shooter.areFlywheelsEnabled());
        }
        telemetry.update();
    }
}