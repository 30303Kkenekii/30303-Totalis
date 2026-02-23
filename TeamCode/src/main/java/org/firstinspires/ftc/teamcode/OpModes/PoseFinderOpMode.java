package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Field Pose Finder", group = "Calibration")
public class PoseFinderOpMode extends LinearOpMode {

    private Follower follower;
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        // 1. Initialize Pedro Pathing purely for tracking
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        // 2. Initialize Drive Motors manually to bypass the library's drive crash
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        telemetry.addLine("Ready. Push the bot or use sticks to drive.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update tracking
            follower.update();

            // 3. Simple Manual Drive (Bypassing the library functions that crash)
            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;

            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            // 4. Get and display Pose
            Pose p = follower.getPose();

            if (p != null) {
                telemetry.addData("X", p.getX());
                telemetry.addData("Y", p.getY());
                telemetry.addData("Heading (Deg)", Math.toDegrees(p.getHeading()));
            } else {
                telemetry.addLine("WAITING FOR DATA...");
            }

            telemetry.update();
        }
    }
}