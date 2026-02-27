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

    public static double blueGoalX = 0, blueGoalY = 144, redGoalX = 144, redGoalY = 144;
    public static double turretOffsetX = -4.5, turretOffsetY = 0;

    // --- YOUR NEWEST REGRESSIONS ---
    public static double sSlope = 5.71543, sIntercept = 601.2141;
    public static double hSlope = 0.00483833, hIntercept = 0.214272;
    public static double hRecoilSlope = 0;
    public static double rpmTolerance = 40;
    public boolean isFiring = false;

    // --- AUTO VELOCITY OFFSET ---
    public static int autoRPMOffset = 0;

    public static int tuningRPM = 2000;
    public static double tuningHoodPos = 0.5;
    public static int tuningTurretTicks = -950;

    // --- HOOD RANGE CLIPPING ---
    public static double hoodMinPos = 0.0; // Configurable minimum
    public static double hoodMaxPos = 0.8; // User requested: Prevents going above 18in

    // --- TURRET & PID ---
    public static double tSlope = -5.5617977528;
    public static int turretMin = -1195, turretMax = -700;
    public static double tp = 0.0021, ti = 0, td = 0.00008, tf = 0;
    public static double angleOffset = 180.0, gateOpenPos = 1.0, gateClosedPos = 0.0, gateToFeedDelay = 0.4;

    private boolean flywheelsActive = true;
    private double currentTargetVelocity = 0;
    public static PIDController tpidfController;
    private double lastRawDegrees = 0, totalUnwrappedDegrees = 0;

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
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretLeft.setDirection(CRServo.Direction.FORWARD);
        turretRight.setDirection(CRServo.Direction.FORWARD);
        tpidfController = new PIDController(tp, ti, td);
        lastRawDegrees = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        totalUnwrappedDegrees = lastRawDegrees;
        closeGate();
    }

    public void enableFlywheels() { flywheelsActive = true; }
    public void disableFlywheels() { flywheelsActive = false; setFlywheelVelocity(0); }
    public boolean areFlywheelsEnabled() { return flywheelsActive; }

    public boolean isAtSpeed() {
        if (currentTargetVelocity == 0 || !flywheelsActive) return false;
        return Math.abs(shooterLeft.getVelocity() - currentTargetVelocity) < rpmTolerance;
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double magVel, double thetaVel, double driverOffset, boolean isAuto) {
        double dist = distToGoal(x, y, blue);
        if (dist < 1) return;

        double angleToGoalField = Math.toDegrees(Math.atan2((blue?blueGoalY:redGoalY) - (y + (turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading))), (blue?blueGoalX:redGoalX) - (x + (turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading)))));
        double targetRelative = (angleToGoalField - Math.toDegrees(heading)) + angleOffset + driverOffset;

        double currentAngle = totalUnwrappedDegrees;
        double diff = targetRelative - (currentAngle % 360);
        if (diff > 180) diff -= 360; if (diff < -180) diff += 360;

        setTurretPosition((int) ((totalUnwrappedDegrees + diff) * tSlope));

        if (flywheelsActive) {
            int baseRPM = getRegressionRPM(dist);
            setFlywheelVelocity(isAuto ? baseRPM + autoRPMOffset : baseRPM);
        } else {
            setFlywheelVelocity(0);
        }

        double targetHoodPos = getRegressionHood(dist);

        // --- DYNAMIC RECOIL & CLIPPING ---
        if (isFiring) {
            double rpmError = Math.max(0, currentTargetVelocity - shooterLeft.getVelocity());
            setHoodPosition(targetHoodPos - (rpmError * hRecoilSlope));
        } else {
            setHoodPosition(targetHoodPos);
        }
    }

    public void setFlywheelVelocity(int velocity) {
        currentTargetVelocity = velocity;
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);
    }

    public void setTurretPosition(int targetTicks) {
        tpidfController.setPID(tp, ti, td);
        int currentTicks = getTurretPos();
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double power = Range.clip(tpidfController.calculate(currentTicks, safeTarget) + tf, -0.7, 0.7);
        if (Math.abs(safeTarget - currentTicks) < 5) power = 0;
        turretLeft.setPower(power); turretRight.setPower(power);
    }

    // UPDATED: Now uses configurable hoodMinPos and hoodMaxPos
    public void setHoodPosition(double pos) {
        shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos));
    }

    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }

    public int getTurretPos() {
        double raw = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        double delta = raw - lastRawDegrees;
        if (delta > 180) delta -= 360; else if (delta < -180) delta += 360;
        totalUnwrappedDegrees += delta; lastRawDegrees = raw;
        return (int) (totalUnwrappedDegrees * tSlope);
    }

    public int getRegressionRPM(double d) { return (int) (sSlope * d + sIntercept); }

    // UPDATED: Regression output is also clipped by the safety limits
    public double getRegressionHood(double d) {
        double pos = hSlope * d + hIntercept;
        return Range.clip(pos, hoodMinPos, hoodMaxPos);
    }

    public double distToGoal(double x, double y, boolean blue) {
        return Math.hypot((blue ? blueGoalX : redGoalX) - x, (blue ? blueGoalY : redGoalY) - y);
    }

    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public double getTargetVelocity() { return currentTargetVelocity; }
}