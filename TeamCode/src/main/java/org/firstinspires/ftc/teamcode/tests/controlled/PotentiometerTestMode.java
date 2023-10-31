package org.firstinspires.ftc.teamcode.tests.controlled;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.robot.Potentiometer;

@TeleOp(name = "Potentiometer Test", group = "Controlled Tests")
public class PotentiometerTestMode extends Mode {

    private AnalogInput potentiometer;

    @Override
    public void onInit() {
        potentiometer = hardwareMap.get(AnalogInput.class, HardwareID.ARM_POTENTIOMETER);
    }

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        double voltage = potentiometer.getVoltage();

        telemetry.addData("Potentiometer Voltage", voltage);
        telemetry.addData("Potentiometer Degrees", Potentiometer.getArmRotDegrees(voltage));
    }
}
