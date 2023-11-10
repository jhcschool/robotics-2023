package org.firstinspires.ftc.teamcode.tests.controlled;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;

@TeleOp(name = "Arm PID Test", group = "Controlled Tests")
public class ArmPIDTestMode extends Mode {
    private Arm arm;
    private ArmController armController;
    private double targetAngle = 0.0;
    private GrizzlyGamepad gamepad;

    @Override
    public void onInit() {
        super.onInit();

        arm = new Arm(hardwareMap);
        armController = new ArmController();

        armController.setTargetAngle(targetAngle);

        gamepad = new GrizzlyGamepad(gamepad1);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        gamepad.update();

        double power = armController.update(arm.getAngle());
        arm.setPower(power);

        if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
            armController.setTargetAngle(targetAngle);
        }

        if (gamepad.getButtonAction(Button.DPAD_UP) == ButtonAction.PRESS) {
            targetAngle += 5.0;
        }

        if (gamepad.getButtonAction(Button.DPAD_DOWN) == ButtonAction.PRESS) {
            targetAngle -= 5.0;
        }

        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Degrees", arm.getAngleDegrees());
        telemetry.addData("Target Angle", targetAngle);
    }
}