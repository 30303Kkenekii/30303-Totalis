package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Configurable
public class IntakeSubsystem {
    // --- Configurable Powers ---
    public static double ASlowDown = 1.0, YSlowDown = 0.3, YSlowDownStage2 = 0.8, INTAKE_POWER = 1.0;

    // --- Color Thresholds ---
    public static double GAIN = 35.0, GREEN_MIN_NORM = 0.15, GREEN_DOMINANCE_FACTOR = 1.2;
    public static double PURPLE_MIN_BLUE_NORM = 0.15, PURPLE_BLUE_DOMINANCE_FACTOR = 1.2, PURPLE_MIN_RED_NORM = 0.1;
    public static double EJECT_DELAY_MS = 250.0; // Wait for pixel to clear rollers before reversing

    public enum DetectedColor { GREEN, PURPLE, UNKNOWN }

    private final DcMotorEx intake, transfer;
    private final NormalizedColorSensor colorSensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        if (colorSensor instanceof SwitchableLight) ((SwitchableLight)colorSensor).enableLight(true);
    }

    // Motor Actions based on specific button power profiles
    public void intakeFull() { intake.setPower(0.8); transfer.setPower(1.0); }
    public void intakeEject() { intake.setPower(-YSlowDown); transfer.setPower(YSlowDownStage2); }
    public void intakeCustom() { intake.setPower(ASlowDown); transfer.setPower(0.5); }
    public void intakeReverse() { intake.setPower(-INTAKE_POWER); transfer.setPower(-INTAKE_POWER); }
    public void intakeOff() { intake.setPower(0); transfer.setPower(0); }

    /**
     * Reads and normalizes color sensor data to identify game elements.
     */
    public DetectedColor getDetectedColor() {
        colorSensor.setGain((float)GAIN);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float nR = colors.red / colors.alpha, nG = colors.green / colors.alpha, nB = colors.blue / colors.alpha;

        // Logic: Checks for intensity and color dominance (e.g. Green must be 1.2x larger than Red/Blue)
        if (nG >= GREEN_MIN_NORM && nG > nR * GREEN_DOMINANCE_FACTOR && nG > nB * GREEN_DOMINANCE_FACTOR) return DetectedColor.GREEN;
        if (nB >= PURPLE_MIN_BLUE_NORM && nR >= PURPLE_MIN_RED_NORM && nB > nG * PURPLE_BLUE_DOMINANCE_FACTOR) return DetectedColor.PURPLE;

        return DetectedColor.UNKNOWN;
    }
}