package org.firstinspires.ftc.teamcode.wrist;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class WristController {

    // For lower arm angles, the wrist should be at 0 degrees.

    private static double BACKDROP_ANGLE = Math.toRadians(360 - 60);
    private static double ARM_THRESHOLD = Math.toRadians(110);
    private static double IMPACT_TIME_THRESHOLD = 0.05;
    private static double IMPACT_ANGLE_THRESHOLD = Math.toRadians(5);

    private Wrist wrist;
    private double lastArmAngle = 0.0;
    private ElapsedTime timeSinceCrossProtectionThreshold = null;

    public WristController(Wrist wrist) {
        this.wrist = wrist;
    }

    public void update(double armAngle, double armOffset) {
        double wristProtectionThreshold = IMPACT_ANGLE_THRESHOLD + armOffset + Arm.BASE_ANGLE;
        if (lastArmAngle > wristProtectionThreshold && armAngle < wristProtectionThreshold) {
            timeSinceCrossProtectionThreshold = new ElapsedTime();
        }

        if (timeSinceCrossProtectionThreshold != null && timeSinceCrossProtectionThreshold.seconds() < IMPACT_TIME_THRESHOLD) {
            wrist.setAngle(Math.toRadians(5));
            return;
        } else {
            timeSinceCrossProtectionThreshold = null;
        }


        if (armAngle < (ARM_THRESHOLD)) {
            wrist.setAngle(0.0 - armOffset);
        } else {
            wrist.setAngle(BACKDROP_ANGLE - (armAngle - Arm.BASE_ANGLE));
        }
    }

    public void update(double armAngle) {
        update(armAngle, 0.0);
    }


}
