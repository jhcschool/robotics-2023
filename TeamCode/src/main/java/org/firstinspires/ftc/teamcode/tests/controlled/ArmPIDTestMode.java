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

@TeleOp(name = "Arm PID Test", group = "Controlled Tests")
public class ArmPIDTestMode extends Mode {
    private Arm arm;
    private ArmController armController;
    private double targetAngle = 20.0;
    private GrizzlyGamepad gamepad;
    private MultipleTelemetry telemetry;
    private Wrist wrist;
    private WristController wristController;

    @Override
    public void onInit() {
        super.onInit();

        arm = new Arm(hardwareMap);
        armController = new ArmController();

        armController.setTargetAngle(Math.toRadians(targetAngle));

        gamepad = new GrizzlyGamepad(gamepad1);

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        wrist = new Wrist(hardwareMap);
        wristController = new WristController(wrist);
    }

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        telemetry.update();

        double deltaTime = time.seconds();
        time.reset();

        gamepad.update();

        double power = armController.update(arm.getAngle());
        arm.setPower(power);

        wristController.update(arm.getAngle());

        if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
            armController.setTargetAngle(Math.toRadians(targetAngle));
        }

        if (gamepad.getButton(Button.DPAD_UP)) {
            targetAngle += 30.0 * deltaTime;
        }

        if (gamepad.getButton(Button.DPAD_DOWN)) {
            targetAngle -= 30.0 * deltaTime;
        }

        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Degrees", arm.getAngleDegrees());
        telemetry.addData("Arm Target Degrees", Math.toDegrees(armController.getTargetAngle()));

    }
}