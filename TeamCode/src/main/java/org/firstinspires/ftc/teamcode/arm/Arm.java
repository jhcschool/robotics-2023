package org.firstinspires.ftc.teamcode.arm;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.robot.Potentiometer;

public class Arm {
    private static final double BASE_ANGLE = Math.toRadians(84.3);

    private DcMotorEx armMotor;
    private AnalogInput potentiometer;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, HardwareID.ARM_MOTOR);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        potentiometer = hardwareMap.get(AnalogInput.class, HardwareID.ARM_POTENTIOMETER);
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public double getPower() {
        return armMotor.getPower();
    }

    public double getAngle() {
        return Math.toRadians(270 - Potentiometer.getArmRotDegrees(potentiometer.getVoltage())) - BASE_ANGLE;
    }

    public double getAngleDegrees() {
        return Math.toDegrees(getAngle());
    }
}
