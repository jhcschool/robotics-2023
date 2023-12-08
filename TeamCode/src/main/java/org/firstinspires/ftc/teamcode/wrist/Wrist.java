package org.firstinspires.ftc.teamcode.wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private static final double WRIST_RANGE = Math.toRadians(355);
    private Servo wristServo;

    public Wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "wrist");
        wristServo.setPosition(0.0);
    }

    public void setAngle(double angle) {
        wristServo.setPosition(angle / WRIST_RANGE);
    }

    public double getAngle() {
        return wristServo.getPosition() * WRIST_RANGE;
    }

    public double getAngleDegrees() {
        return Math.toDegrees(getAngle());
    }
}
