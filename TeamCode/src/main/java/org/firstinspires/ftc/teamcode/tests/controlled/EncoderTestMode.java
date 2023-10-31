package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;

@TeleOp(name = "Encoder Test", group = "Controlled Tests")
public class EncoderTestMode extends Mode {
    private Encoder par0;
    private Encoder par1;
    private Encoder perp;

    @Override
    public void onInit() {
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, HardwareID.LEFT_FRONT_MOTOR)));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, HardwareID.LEFT_BACK_MOTOR)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, HardwareID.RIGHT_BACK_MOTOR)));
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        telemetry.addData("Parallel 0", par0.getPositionAndVelocity().position);
        telemetry.addData("Parallel 1", par1.getPositionAndVelocity().position);
        telemetry.addData("Perpendicular", perp.getPositionAndVelocity().position);
    }
}
