package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.input.Axis;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;

@TeleOp(name = "Basic Control Test", group = "Controlled Tests")
public class BasicControlTestMode extends Mode {
    private GrizzlyGamepad gamepad;
    private MecanumDrive drive;

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        float forward = -gamepad.getAxis(Axis.LEFT_STICK_Y);
        float right = -gamepad.getAxis(Axis.LEFT_STICK_X);
        float rotation = -gamepad.getAxis(Axis.RIGHT_STICK_X);

        PoseVelocity2d powers = new PoseVelocity2d(new Vector2d(forward, right), rotation);
        drive.setDrivePowers(powers);
    }
}
