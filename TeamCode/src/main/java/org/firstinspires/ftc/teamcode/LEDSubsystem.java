package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class LEDSubsystem {
    private final Servo indicator;
    public static double COLOR_RED = 0.279, COLOR_GREEN = 0.500;
    public static double MAX_ERROR = 500.0;

    public LEDSubsystem(HardwareMap hardwareMap) {
        indicator = hardwareMap.get(Servo.class, "LEDindicator");
    }

    public void updateRPMIndicator(double actual, double target) {
        double pct = Math.min(1.0, Math.abs(actual - target) / MAX_ERROR);
        indicator.setPosition(COLOR_GREEN - (pct * (COLOR_GREEN - COLOR_RED)));
    }
}