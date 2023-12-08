package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.claw.Claw;

@TeleOp(name = "Claw Test", group = "Controlled Tests")
public class ClawTestMode extends Mode {
    private Claw claw;

    @Override
    public void onInit() {
        super.onInit();

        claw = new Claw(hardwareMap);
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        if (gamepad1.a) {
            claw.openLeft();
        } else if (gamepad1.b) {
            claw.closeLeft();
        }

        if (gamepad1.x) {
            claw.openRight();
        } else if (gamepad1.y) {
            claw.closeRight();
        }

        telemetry.addData("Left Claw Open", claw.isLeftOpen());
        telemetry.addData("Right Claw Open", claw.isRightOpen());
    }


}
