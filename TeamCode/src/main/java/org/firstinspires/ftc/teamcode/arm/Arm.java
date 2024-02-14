package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.wrist.WristController;

public class Arm {
    private static final double TICKS_PER_REV = 8192;
    public static double BASE_ANGLE = Math.toRadians(-26.2);

    private DcMotorEx armMotor;
    private Encoder armEncoder;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, HardwareID.ARM_MOTOR);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armEncoder = new RawEncoder(armMotor);

        resetEncoder();
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public double getPower() {
        return armMotor.getPower();
    }

    public double getAngle() {
        return (armEncoder.getPositionAndVelocity().position / TICKS_PER_REV) * 2 * Math.PI + BASE_ANGLE;
    }

    public double getAngleDegrees() {
        return Math.toDegrees(getAngle());
    }

    public double getVelocity() {
        return (armEncoder.getPositionAndVelocity().velocity / TICKS_PER_REV) * 2 * Math.PI;
    }

    public void resetEncoder() {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
