package org.firstinspires.ftc.teamcode.wrist;

import org.firstinspires.ftc.teamcode.arm.Arm;

public class WristController {

    // For lower arm angles, the wrist should be at 0 degrees.

    private static double BACKDROP_ANGLE = Math.toRadians(360 - 60);
    private static double ARM_THRESHOLD = Math.toRadians(110);
    private Wrist wrist;

    public WristController(Wrist wrist) {
        this.wrist = wrist;
    }

    public void update(double armAngle) {
        if (armAngle < (ARM_THRESHOLD)) {
            wrist.setAngle(0.0);
        } else {
            wrist.setAngle(BACKDROP_ANGLE - (armAngle - Arm.BASE_ANGLE));
        }
    }


}
