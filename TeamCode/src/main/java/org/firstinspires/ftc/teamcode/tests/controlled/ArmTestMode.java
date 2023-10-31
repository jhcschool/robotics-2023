package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.robot.Potentiometer;

@TeleOp(name = "Arm Test", group = "Controlled Tests")
public class ArmTestMode extends Mode {
    private DcMotorEx armMotor;
    private AnalogInput potentiometer;

    @Override
    public void onInit() {
        super.onInit();

        armMotor = hardwareMap.get(DcMotorEx.class, HardwareID.ARM_MOTOR);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        potentiometer = hardwareMap.get(AnalogInput.class, HardwareID.ARM_POTENTIOMETER);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        armMotor.setPower(power);

        telemetry.addData("Arm Power", power);

        double position = armMotor.getCurrentPosition();

        telemetry.addData("Arm Encoder", position);
        telemetry.addData("Arm Degrees", Potentiometer.getArmRotDegrees(potentiometer.getVoltage()));

    }
}
