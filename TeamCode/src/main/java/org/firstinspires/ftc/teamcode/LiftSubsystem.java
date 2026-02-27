package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class LiftSubsystem {
    private final CRServo liftLeft, liftRight;

    // Configurable power for the lift
    public static double LIFT_POWER = 1.0;

    public LiftSubsystem(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(CRServo.class, "liftLeft");
        liftRight = hardwareMap.get(CRServo.class, "liftRight");

        // Set direction: liftRight needs reversal as requested
        liftLeft.setDirection(CRServo.Direction.FORWARD);
        liftRight.setDirection(CRServo.Direction.REVERSE);

        // Ensure they start at 0
        stop();
    }

    /**
     * Actuates the lift upwards using the configured LIFT_POWER.
     */
    public void liftUp() {
        liftLeft.setPower(LIFT_POWER);
        liftRight.setPower(LIFT_POWER);
    }

    /**
     * Actuates the lift downwards.
     */
    public void liftDown() {
        liftLeft.setPower(-LIFT_POWER);
        liftRight.setPower(-LIFT_POWER);
    }

    /**
     * Stops the CR Servos immediately.
     */
    public void stop() {
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }
}