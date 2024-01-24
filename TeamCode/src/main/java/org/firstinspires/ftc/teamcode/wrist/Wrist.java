package org.firstinspires.ftc.teamcode.wrist;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {
    public static final double WRIST_RANGE = Math.toRadians(355) * 0.9;
    public static final double WRIST_MIN = Math.toRadians(-10);
    private Servo wristServo;

    public Wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

    public void setAngle(double angle) {
        wristServo.setPosition((angle - WRIST_MIN) / WRIST_RANGE);
    }

    public double getAngle() {
        return wristServo.getPosition() * WRIST_RANGE;
    }

    public double getAngleDegrees() {
        return Math.toDegrees(getAngle());
    }
}
