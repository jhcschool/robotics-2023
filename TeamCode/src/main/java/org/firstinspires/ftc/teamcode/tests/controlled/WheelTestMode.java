package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;

@TeleOp(name = "Wheel Test", group = "Controlled Tests")
public class WheelTestMode extends Mode {

    private final DcMotorEx[] motors = new DcMotorEx[4];
    private int motorIndex;

    private static String[] MOTOR_NAMES = {HardwareID.LEFT_FRONT_MOTOR, HardwareID.LEFT_BACK_MOTOR, HardwareID.RIGHT_BACK_MOTOR, HardwareID.RIGHT_FRONT_MOTOR};
    private static boolean[] REVERSED = {false, false, false, false};

    @Override
    public void onInit() {
        super.onInit();

        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            if (REVERSED[i]) {
                motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        if (gamepad1.dpad_up) {
            motorIndex = 0;
        } else if (gamepad1.dpad_right) {
            motorIndex = 1;
        } else if (gamepad1.dpad_down) {
            motorIndex = 2;
        } else if (gamepad1.dpad_left) {
            motorIndex = 3;
        }


        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        motors[motorIndex].setPower(power);
        telemetry.addData("Motor", MOTOR_NAMES[motorIndex]);
        telemetry.addData("Power", power);
    }

}
