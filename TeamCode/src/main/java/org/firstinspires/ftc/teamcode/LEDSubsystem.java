package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class LEDSubsystem {
    private final Servo indicator;
    // Standard goBILDA RGB positions
    public static double COLOR_RED = 0.279, COLOR_GREEN = 0.500, COLOR_BLUE = 0.611, COLOR_WHITE = 1.0;
    public static double MAX_ERROR = 500.0; // Error value that results in pure Red

    public LEDSubsystem(HardwareMap hardwareMap) {
        indicator = hardwareMap.get(Servo.class, "LEDindicator");
    }

    /**
     * Slides the LED color from Green (Zero Error) to Red (Max Error).
     * This helps the driver know when the flywheels have recovered.
     */
    public void updateRPMIndicator(double actualRPM, double targetRPM) {
        double pct = Math.min(1.0, Math.abs(actualRPM - targetRPM) / MAX_ERROR);
        // Linear Interpolation (Lerp) between 0.5 and 0.279
        setColor(COLOR_GREEN - (pct * (COLOR_GREEN - COLOR_RED)));
    }

    public void setColor(double p) { indicator.setPosition(p); }
}
