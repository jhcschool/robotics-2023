package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.ButtonAction;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

import java.util.HashMap;

@TeleOp(name = "Arm Time Test", group = "Controlled Tests")
public class ArmTimeTestMode extends Mode {
    private final HashMap<Modifier, Double> modifiers = new HashMap<Modifier, Double>() {{
        put(Modifier.RESET_TIME, 1.0);
        put(Modifier.UP, 0.6);
        put(Modifier.TIME, 0.2);
    }};

    private Arm arm;
    private Wrist wrist;
    private GrizzlyGamepad gamepad;

    private ElapsedTime elapsedTime;
    private Modifier modifier = Modifier.TIME;
    private State state = State.MAINTAIN;

    @Override
    public void onInit() {
        super.onInit();

        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);

        gamepad = new GrizzlyGamepad(gamepad1);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);
        gamepad.update();

        double armAngle = arm.getAngle();

        double wristAngleToMaintainZero = -armAngle - Arm.BASE_ANGLE;
        if (wristAngleToMaintainZero < Wrist.WRIST_MIN) {
            wristAngleToMaintainZero = 0;
        }

        wrist.setAngle(wristAngleToMaintainZero);

        switch (state) {
            case MAINTAIN:
                arm.setPower(ArmController.feedforward(armAngle));
                break;
            case MOVING:
                arm.setPower(modifiers.get(Modifier.UP));

                if (elapsedTime.seconds() > modifiers.get(Modifier.TIME)) {
                    moveToState(State.MAINTAIN);
                }
                break;
            case RESET:
                arm.setPower(-0.2);
                if (elapsedTime.seconds() > modifiers.get(Modifier.RESET_TIME)) {
                    moveToState(State.MAINTAIN);
                }
                break;
        }

        if (state == State.MAINTAIN) {
            if (gamepad.getButtonAction(Button.A) == ButtonAction.PRESS) {
                moveToState(State.MOVING);
            }

            if (gamepad.getButtonAction(Button.X) == ButtonAction.PRESS) {
                moveToState(State.RESET);
            }
        }

        if (gamepad.getButtonAction(Button.DPAD_RIGHT) == ButtonAction.PRESS) {
            modifiers.put(modifier, modifiers.get(modifier) + 0.01);
        } else if (gamepad.getButtonAction(Button.DPAD_LEFT) == ButtonAction.PRESS) {
            modifiers.put(modifier, modifiers.get(modifier) - 0.01);
        }
    }

    private void moveToState(State state) {
        elapsedTime.reset();
        this.state = state;
    }

    private enum State {
        MAINTAIN,
        RESET,
        MOVING
    }

    private enum Modifier {
        RESET_TIME,
        UP,
        TIME
    }
}
