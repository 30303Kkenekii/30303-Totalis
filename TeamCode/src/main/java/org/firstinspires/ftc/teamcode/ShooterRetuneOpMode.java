package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Flywheel & Hood Tuning", group = "TeleOp")
public class ShooterRetuneOpMode extends OpMode {
    public TelemetryManager teleM;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    // Raw Drive Motors (Robot Centric to prevent crashes)
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Static variables for Dashboard/Tuning
    public static int vel = 0;
    public static double hoodPos = 0.5;

    private double fError;
    private boolean lastX = false, lastA = false, lastLB = false, lastRB = false;

    @Override
    public void init() {
        // --- Subsystems ---
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        // --- Drive Motors ---
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

        // --- Telemetry ---
        teleM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter.disableFlywheels();
        shooter.openGate();
        intake.intakeOff();
    }

    @Override
    public void loop() {
        follower.update();
        Pose cp = follower.getPose();

        // --- 1. ROBOT CENTRIC DRIVE ---
        // Translation: Right Stick | Rotation: Left Stick
        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / den);
        leftBack.setPower((y - x + rx) / den);
        rightFront.setPower((y - x - rx) / den);
        rightBack.setPower((y + x - rx) / den);

        // --- 2. INPUT HANDLING ---

        // Velocity (Bumpers)
        if (gamepad1.right_bumper && !lastRB) {
            vel += 100;
            gamepad1.rumble(100);
        } else if (gamepad1.left_bumper && !lastLB) {
            vel -= 100;
            gamepad1.rumble(100);
        }

        // Hood (D-Pad)
        if (gamepad1.dpad_up) hoodPos += 0.001;
        if (gamepad1.dpad_down) hoodPos -= 0.001;
        hoodPos = Math.max(0, Math.min(1, hoodPos));

        // Kick/Feed (A - Held)
        if (gamepad1.a) {
            shooter.enableFlywheels();
        } else if (!gamepad1.a && lastA) {
            intake.intakeOff();
        }

        // Intake Toggle (X)
        if (gamepad1.x && !lastX) {
            intake.intakeFull();
        }

        // STOP (B)
        if (gamepad1.b) {
            intake.intakeOff();
            shooter.disableFlywheels();
        }

        lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper;
        lastA = gamepad1.a; lastX = gamepad1.x;

        // --- 3. HARDWARE UPDATES ---
        if (vel > 0) {
            shooter.setFlywheelVelocity(vel);
        }

        shooter.setHoodPosition(hoodPos);

        // Hold Turret at the Forward position for distance tuning
        shooter.setTurretPosition(ShooterSubsystem.tuningTurretTicks);

        // --- 4. DATA TELEMETRY ---
        if (cp != null) {
            // Distance to goal (using Blue Goal coordinates 0, 144)
            double dist = Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(), ShooterSubsystem.blueGoalY - cp.getY());
            fError = Math.abs(vel - shooter.getCurrentVelocity());

            teleM.addData("--- TUNING DATA ---", "");
            teleM.addData("Current DISTANCE", String.format("%.2f", dist));
            teleM.addData("Target Velocity", vel);
            teleM.addData("Actual Velocity", (int)shooter.getCurrentVelocity());
            teleM.addData("Velocity Error", (int)fError);
            teleM.addData("Hood Position", String.format("%.3f", hoodPos));
            teleM.addData("Robot Pose", String.format("X:%.1f Y:%.1f", cp.getX(), cp.getY()));
        } else {
            teleM.addData("STATUS", "WAITING FOR LOCALIZER...");
        }

        teleM.update();
    }
}