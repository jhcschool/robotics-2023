package org.firstinspires.ftc.teamcode.tests.controlled;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

@TeleOp(name = "Arm Precision Test", group = "Controlled Tests")
public class ArmPrecisionTestMode extends Mode {
    private static final double FLIPPED_ANGLE = Math.toRadians(150);

    private Arm arm;
    private ArmController armController;
    private double baseOffset = 5;
    private GrizzlyGamepad gamepad;
    private boolean flipped = false;
    private Wrist wrist;
    private WristController wristController;

    @Override
    public void onInit() {
        super.onInit();

        arm = new Arm(hardwareMap);
        armController = new ArmController();

        armController.setTargetAngle(Math.toRadians(baseOffset) + Arm.BASE_ANGLE);

        gamepad = new GrizzlyGamepad(gamepad1);

        wrist = new Wrist(hardwareMap);
        wristController = new WristController(wrist);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        gamepad.update();

        double power = armController.update(arm.getAngle());
        arm.setPower(power);

        wristController.update(arm.getAngle(), Math.toRadians(baseOffset));

        if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
            flipped = !flipped;
            armController.setTargetAngle(flipped ? FLIPPED_ANGLE : Math.toRadians(baseOffset) + Arm.BASE_ANGLE);
        }

        if (gamepad.getButtonAction(Button.DPAD_UP) == ButtonAction.PRESS) {
            baseOffset += 1.0;
        }

        if (gamepad.getButtonAction(Button.DPAD_DOWN) == ButtonAction.PRESS) {
            baseOffset -= 1.0;
        }

        telemetry.addData("Offset", baseOffset);
        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Degrees", arm.getAngleDegrees());
        telemetry.addData("Arm Target Degrees", Math.toDegrees(armController.getTargetAngle()));
        telemetry.addData("Wrist Degrees", wrist.getAngleDegrees());
    }
}