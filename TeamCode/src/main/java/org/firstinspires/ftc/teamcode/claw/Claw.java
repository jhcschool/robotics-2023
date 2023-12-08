package org.firstinspires.ftc.teamcode.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo leftClawServo;
    private Servo rightClawServo;

    public Claw(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, "leftClaw");
        rightClawServo = hardwareMap.get(Servo.class, "rightClaw");

        leftClawServo.setPosition(0.0);
        rightClawServo.setPosition(0.0);
    }

    public void openLeft() {
        leftClawServo.setPosition(0.0);
    }

    public void openRight() {
        rightClawServo.setPosition(1.0);
    }

    public void closeLeft() {
        leftClawServo.setPosition(1.0);
    }

    public void closeRight() {
        rightClawServo.setPosition(0.0);
    }

    public boolean isLeftOpen() {
        return leftClawServo.getPosition() == 0.0;
    }

    public boolean isRightOpen() {
        return rightClawServo.getPosition() == 1.0;
    }
}
