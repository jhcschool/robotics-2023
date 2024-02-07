package org.firstinspires.ftc.teamcode.wrist;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class WristController {

    // For lower arm angles, the wrist should be at 0 degrees.

    private static double BACKDROP_ANGLE = Math.toRadians(360 - 60);
    private static double ARM_THRESHOLD = Math.toRadians(95);
    private static double IMPACT_TIME_THRESHOLD = 0.2;
    private static double IMPACT_ANGLE_THRESHOLD = Math.toRadians(9);

    private Wrist wrist;
    private double lastArmAngle = Arm.BASE_ANGLE;
    private ElapsedTime timeSinceCrossProtectionThreshold = new ElapsedTime();

    public WristController(Wrist wrist) {
        this.wrist = wrist;
    }

    public void update(double armAngle, double armOffset) {
        double wristProtectionThreshold = IMPACT_ANGLE_THRESHOLD + armOffset + Arm.BASE_ANGLE;
        if (lastArmAngle > wristProtectionThreshold && armAngle < wristProtectionThreshold) {
            timeSinceCrossProtectionThreshold = new ElapsedTime();
        } else if (lastArmAngle < wristProtectionThreshold && armAngle > wristProtectionThreshold) {
            timeSinceCrossProtectionThreshold = null;
        }

        lastArmAngle = armAngle;

        if (timeSinceCrossProtectionThreshold != null && timeSinceCrossProtectionThreshold.seconds() > IMPACT_TIME_THRESHOLD) {
            wrist.setAngle(0.0 - armOffset);
            return;
        }


        if (armAngle < (ARM_THRESHOLD)) {
            wrist.setAngle(Math.toRadians(35));
        } else {
            wrist.setAngle(BACKDROP_ANGLE - (armAngle - Arm.BASE_ANGLE));
        }
    }

    public void update(double armAngle) {
        update(armAngle, 0.0);
    }


}
