package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Master SOTM Regression Test", group = "A")
public class SimpleTurretTracker extends LinearOpMode {

    private Follower follower;
    private ShooterSubsystem shooter;
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private boolean isBlue = true;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Raw motor drive for safety
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        follower.setStartingPose(new Pose(8, 8, 0));

        while (opModeInInit()) {
            if (gamepad1.b) isBlue = !isBlue;
            telemetry.addData("Target Goal", isBlue ? "BLUE" : "RED");
            telemetry.addData("Init Ticks", shooter.getTurretPos());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose();
            Vector vv = follower.getVelocity();

            // Safe Drive (Robot Centric)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            if (cp != null) {
                // Tracking + Physics
                //shooter.alignTurret(cp.getX(), cp.getY(), cp.getHeading(), isBlue, telemetry, vv.getMagnitude(), vv.getTheta());
            }

            telemetry.update();
        }
    }
}