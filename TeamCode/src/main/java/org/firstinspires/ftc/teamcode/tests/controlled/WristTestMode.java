package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

@TeleOp(name = "Wrist Test", group = "Controlled Tests")
public class WristTestMode extends Mode {

    private GrizzlyGamepad gamepad;
    private Wrist wrist;
    private double targetAngle = 0;
    @Override
    public void onInit() {
        super.onInit();
        wrist = new Wrist(hardwareMap);

        gamepad = new GrizzlyGamepad(gamepad1);
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        gamepad.update();

        if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
            wrist.setAngle(Math.toRadians(targetAngle));
        }

        if (gamepad.getButtonAction(Button.DPAD_UP) == ButtonAction.PRESS) {
            targetAngle += 5.0;
        }

        if (gamepad.getButtonAction(Button.DPAD_DOWN) == ButtonAction.PRESS) {
            targetAngle -= 5.0;
        }

        telemetry.addData("Wrist Degrees", wrist.getAngleDegrees());
        telemetry.addData("Target Angle", targetAngle);
    }
}
