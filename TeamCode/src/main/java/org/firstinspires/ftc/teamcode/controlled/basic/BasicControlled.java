package org.firstinspires.ftc.teamcode.controlled.basic;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.input.Axis;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.robot.HardwareID;

@TeleOp(name = "Basic Controlled", group = "Controlled")
public class BasicControlled extends Mode {

    private static final double INITIAL_CLAW_POSITION = 0.5;
    private static final double FINAL_CLAW_POSITION = 0.35;

    private boolean clawOpen = true;
    private GrizzlyGamepad gamepad;
    private MecanumDrive drive;
    private Arm arm;
    private Servo clawServo;

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm = new Arm(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setPosition(INITIAL_CLAW_POSITION);
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);
        gamepad.update();

        drive.updatePoseEstimate();

        float forward = -gamepad.getAxis(Axis.LEFT_STICK_Y);
        float right = -gamepad.getAxis(Axis.LEFT_STICK_X);
        float rotation = -gamepad.getAxis(Axis.RIGHT_STICK_X);

        PoseVelocity2d powers = new PoseVelocity2d(new Vector2d(forward, right), rotation);
        drive.setDrivePowers(powers);

        double time = timer.seconds();
        double dt = time - lastTime;
        lastTime = time;

        telemetry.addData("Frame time", dt);

        double power = gamepad.getAxis(Axis.RIGHT_TRIGGER) - gamepad.getAxis(Axis.LEFT_TRIGGER);
        arm.setPower(power);

        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Angle", arm.getAngleDegrees());

        if (gamepad.getButtonAction(Button.RIGHT_BUMPER) == ButtonAction.PRESS) {
            clawOpen = !clawOpen;
            clawServo.setPosition(clawOpen ? INITIAL_CLAW_POSITION : FINAL_CLAW_POSITION);
        }

        telemetry.addData("Claw Open", clawOpen);
    }
}
