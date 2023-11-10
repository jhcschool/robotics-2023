package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.robot.Potentiometer;

@TeleOp(name = "Servo Test", group = "Controlled Tests")
public class ServoTestMode extends Mode {
    private Servo servo;
    private double servoPosition = 0.5;
    private GrizzlyGamepad gamepad;

    @Override
    public void onInit() {
        super.onInit();

        servo = hardwareMap.get(Servo.class, "wristServo");
        servo.setPosition(servoPosition);

        gamepad = new GrizzlyGamepad(gamepad1);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);
        gamepad.update();

        if (gamepad.getButtonAction(Button.DPAD_UP) == ButtonAction.PRESS) {
            servoPosition += 0.01;
        }

        if (gamepad.getButtonAction(Button.DPAD_DOWN) == ButtonAction.PRESS) {
            servoPosition -= 0.01;
        }

        servoPosition = Math.min(Math.max(servoPosition, 0), 1);

        telemetry.addData("Servo Position", servoPosition);
        servo.setPosition(servoPosition);

    }
}
