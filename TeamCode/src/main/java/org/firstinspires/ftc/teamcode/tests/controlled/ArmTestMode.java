package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.robot.Potentiometer;

@TeleOp(name = "Arm Test", group = "Controlled Tests")
public class ArmTestMode extends Mode {
    private Arm arm;

    @Override
    public void onInit() {
        super.onInit();

        arm = new Arm(hardwareMap);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        arm.setPower(power);

        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Degrees", arm.getAngleDegrees());

    }
}
