package org.firstinspires.ftc.teamcode.wrist;

public class WristController {

    // For lower arm angles, the wrist should be at 0 degrees.

    private static double BACKDROP_ANGLE = Math.toRadians(360 - 60);
    private static double ARM_THRESHOLD = Math.toRadians(60);
    private static double ARM_ANGLE_BASE_TO_LEVEL = Math.toRadians(0);
    private Wrist wrist;

    public WristController(Wrist wrist) {
        this.wrist = wrist;
    }

    public void update(double armAngleFromBase) {
        if (armAngleFromBase < (ARM_THRESHOLD + ARM_ANGLE_BASE_TO_LEVEL)) {
            wrist.setAngle(0.0);
        } else {
            wrist.setAngle(BACKDROP_ANGLE - (armAngleFromBase + ARM_ANGLE_BASE_TO_LEVEL));
        }
    }


}
