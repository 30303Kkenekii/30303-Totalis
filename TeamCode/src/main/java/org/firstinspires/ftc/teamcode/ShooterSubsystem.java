package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    private DcMotorEx shooterLeft, shooterRight;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;
    private AnalogInput turretEncoderLeft;

    public static double blueGoalX = 5, blueGoalY = 144, redGoalX = 144, redGoalY = 136;
    public static double turretOffsetX = -4.5, turretOffsetY = 0;

    // --- NEW LINEAR RPM REGRESSION ---
    public static double sSlope = 5.95999;
    public static double sIntercept = 533.71646;

    // --- NEW QUARTIC HOOD REGRESSION ---
    public static double hA = -2.21341e-9;
    public static double hB = 0.00000111483;
    public static double hC = -0.000206224;
    public static double hD = 0.0183896;
    public static double hE = -0.0778329;

    // --- HARDWARE & PID ---
    public static double tSlope = -5.5617977528;
    public static int turretMin = -4000, turretMax = 4000;
    public static double tp = 0.002, ti = 0, td = 0.00005, tf = 0;
    public static double angleOffset = 180.0;
    public static double gateOpenPos = 1.0, gateClosedPos = 0.0;
    public static double gateToFeedDelay = 0.25;
    public static boolean flywheelsEnabled = true;
    public static double timeInAirB = 0.6, timeInAirM = 0.0012;

    // Tuning Sliders
    public static int tuningRPM = 2000;
    public static double tuningHoodPos = 0.5;
    public static int tuningTurretTicks = -950;

    private double currentTargetVelocity = 0;
    public static PIDController tpidfController;
    private double lastRawDegrees = 0;
    private double totalUnwrappedDegrees = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        shooterHood = hardwareMap.get(Servo.class, "shooterHood");
        shooterGate = hardwareMap.get(Servo.class, "shooterGate");
        turretEncoderLeft = hardwareMap.get(AnalogInput.class, "turretEncoderLeft");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        turretLeft.setDirection(CRServo.Direction.FORWARD);
        turretRight.setDirection(CRServo.Direction.FORWARD);

        tpidfController = new PIDController(tp, ti, td);
        lastRawDegrees = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        totalUnwrappedDegrees = lastRawDegrees;
        closeGate();
    }

    public int getRegressionRPM(double distance) {
        // y = mx + b
        return (int) (sSlope * distance + sIntercept);
    }

    public double getRegressionHood(double d) {
        // y = ax^4 + bx^3 + cx^2 + dx + e
        return Range.clip(hA*Math.pow(d,4) + hB*Math.pow(d,3) + hC*Math.pow(d,2) + hD*d + hE, 0, 1);
    }

    public void setTurretPosition(int targetTicks) {
        tpidfController.setPID(tp, ti, td);
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double power = Range.clip(tpidfController.calculate(getTurretPos(), safeTarget) + tf, -0.7, 0.7);
        if (Math.abs(safeTarget - getTurretPos()) < 5) power = 0;
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double magVel, double thetaVel, double driverOffset) {
        double dist = distToGoal(x, y, blue);
        if (dist < 1) return;

        currentTargetVelocity = getRegressionRPM(dist);
        double timeInAir = timeInAirM * currentTargetVelocity - timeInAirB;
        if (timeInAir < 0) timeInAir = 0;

        double predX = (blue ? blueGoalX : redGoalX) - (magVel * Math.cos(thetaVel)) * timeInAir;
        double predY = (blue ? blueGoalY : redGoalY) - (magVel * Math.sin(thetaVel)) * timeInAir;

        double targetRobotRelative = (Math.toDegrees(Math.atan2(predY - y, predX - x)) - Math.toDegrees(heading)) + angleOffset + driverOffset;
        double diff = targetRobotRelative - (totalUnwrappedDegrees % 360);
        if (diff > 180) diff -= 360; if (diff < -180) diff += 360;

        setTurretPosition((int) ((totalUnwrappedDegrees + diff) * tSlope));

        double finalDist = Math.hypot(predX - x, predY - y);
        if (flywheelsEnabled) setFlywheelVelocity(getRegressionRPM(finalDist));
        else setFlywheelVelocity(0);

        shooterHood.setPosition(getRegressionHood(finalDist));
    }

    public void setFlywheelVelocity(int velocity) {
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);
    }

    public void setHoodPosition(double pos) { shooterHood.setPosition(Range.clip(pos, 0, 1)); }
    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public int getTurretPos() {
        double raw = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        double delta = raw - lastRawDegrees;
        if (delta > 180) delta -= 360; else if (delta < -180) delta += 360;
        totalUnwrappedDegrees += delta; lastRawDegrees = raw;
        return (int) (totalUnwrappedDegrees * tSlope);
    }
    public double distToGoal(double x, double y, boolean blue) {
        return Math.hypot((blue ? blueGoalX : redGoalX) - x, (blue ? blueGoalY : redGoalY) - y);
    }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public double getTargetVelocity() { return currentTargetVelocity; }
}