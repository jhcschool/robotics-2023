package org.firstinspires.ftc.teamcode.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.HardwareID;

public class Claw {
    private Servo leftClawServo;
    private Servo rightClawServo;

    public Claw(HardwareMap hardwareMap) {
        leftClawServo = hardwareMap.get(Servo.class, HardwareID.LEFT_CLAW);
        rightClawServo = hardwareMap.get(Servo.class, HardwareID.RIGHT_CLAW);

        leftClawServo.setPosition(0.0);
        rightClawServo.setPosition(0.0);
    }

    public void openLeft() {
        leftClawServo.setPosition(1.0);
    }

    public void openRight() {
        rightClawServo.setPosition(0.0);
    }

    public void closeLeft() {
        leftClawServo.setPosition(0.0);
    }

    public void closeRight() {
        rightClawServo.setPosition(1.0);
    }

    public boolean isLeftOpen() {
        return leftClawServo.getPosition() == 1.0;
    }

    public boolean isRightOpen() {
        return rightClawServo.getPosition() == 0.0;
    }
}
