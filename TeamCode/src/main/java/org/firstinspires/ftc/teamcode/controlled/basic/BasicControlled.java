package org.firstinspires.ftc.teamcode.controlled.basic;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.input.Axis;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

@Config
@TeleOp(name = "Basic Controlled", group = "Controlled")
public class BasicControlled extends Mode {
    public static double K_COS = 0.10;
    public static double DEADZONE = 0;
    public static double ANGLE_WEIGHTING = 0.6;
    private boolean clawOpen = true;
    private GrizzlyGamepad gamepad;
    private MecanumDrive drive;
    private Arm arm;
    private Claw claw;
    private Wrist wrist;
    private WristController wristController;
    private boolean clamp = false;
    private boolean holdingClampToggle = false;

    private enum ControlReferenceMode {
        ROBOT,
        FIELD
    }

    private ControlReferenceMode controlReferenceMode = ControlReferenceMode.ROBOT;

    private ElapsedTime timer;
    private double lastTime = 0;

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        wrist.setAngle(0);
        wristController = new WristController(wrist);

        claw.openLeft();
        claw.openRight();

        timer = new ElapsedTime();

        setGamepadLed();
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);
        gamepad.update();

        drive.updatePoseEstimate();

        float forward = -gamepad.getAxis(Axis.LEFT_STICK_Y);
        float right = -gamepad.getAxis(Axis.LEFT_STICK_X);
        float rotation = -gamepad.getAxis(Axis.RIGHT_STICK_X);

        PoseVelocity2d powers;
        if (controlReferenceMode == ControlReferenceMode.FIELD) {
            Vector2d nonRotated = new Vector2d(forward, right);
            powers = new PoseVelocity2d(drive.pose.heading.inverse().times(nonRotated), rotation);
        } else {
            powers = new PoseVelocity2d(new Vector2d(forward, right), rotation);
        }

        drive.setDrivePowers(powers);

        double time = timer.seconds();
        double dt = time - lastTime;
        lastTime = time;

        telemetry.addData("Frame time", dt);

        double angle = arm.getAngle();

        double power = gamepad.getAxis(Axis.RIGHT_TRIGGER) - gamepad.getAxis(Axis.LEFT_TRIGGER);
        if (clamp) {
            power = -1.0;
        } else {
            if (Math.abs(angle - Math.toRadians(90)) < Math.toRadians(DEADZONE)) {
                power = 0.0;
            }

            power -= power * ANGLE_WEIGHTING * Math.abs(Math.sin(angle));
            power += Math.cos(angle) * K_COS;

            power = Math.max(-1, Math.min(1, power));
        }
        arm.setPower(power);


        telemetry.addData("Arm Power", power);
        telemetry.addData("Arm Angle", Math.toDegrees(angle));

        wristController.update(angle);

        if (gamepad.getButtonAction(Button.RIGHT_BUMPER) == ButtonAction.PRESS) {
            clawOpen = !clawOpen;

            if (clawOpen) {
                claw.openLeft();
                claw.openRight();
            } else {
                claw.closeLeft();
                claw.closeRight();
            }
        }

        if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
            int newValue = controlReferenceMode.ordinal() + 1;
            if (newValue >= ControlReferenceMode.values().length) {
                newValue = 0;
            }

            controlReferenceMode = ControlReferenceMode.values()[newValue];
            setGamepadLed();
        }

        if (gamepad.getButton(Button.Y) && gamepad.getButton(Button.DPAD_UP)) {
            if (!holdingClampToggle) {
                clamp = !clamp;
                holdingClampToggle = true;
            }
        } else {
            holdingClampToggle = false;
        }

        telemetry.addData("Claw Open", clawOpen);
        telemetry.addData("Wrist Angle", wrist.getAngleDegrees());

        telemetry.addData("Control Reference Mode", controlReferenceMode);
    }

    private void setGamepadLed() {
        if (controlReferenceMode == ControlReferenceMode.ROBOT) {
            gamepad.setLed(255, 0, 0);
        } else {
            gamepad.setLed(0, 0, 255);
        }
    }
}
