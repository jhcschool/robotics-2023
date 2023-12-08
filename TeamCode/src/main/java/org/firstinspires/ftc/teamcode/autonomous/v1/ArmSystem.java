package org.firstinspires.ftc.teamcode.autonomous.v1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;

@Config
public class ArmSystem {
    private static double RAISED_ANGLE = Math.toRadians(135);
    private static double BASE_ANGLE = Math.toRadians(0);
    private static double ERROR_THRESHOLD = Math.toRadians(5);

    private Arm arm;

    public ArmSystem(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
    }

    private class ArmAction implements Action {
        private ArmController armController;

        public ArmAction(double targetAngle) {
            armController = new ArmController();
            armController.setTargetAngle(targetAngle);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (Math.abs(armController.getTargetAngle() - arm.getAngle()) < ERROR_THRESHOLD) {
                arm.setPower(0.0);
                return false;
            }

            arm.setPower(armController.update(arm.getAngle()));
            return true;
        }
    }

    public Action raiseArm() {
        return new ArmAction(RAISED_ANGLE);
    }

    public Action lowerArm() {
        return new ArmAction(BASE_ANGLE);
    }

    public double getAngle() {
        return arm.getAngle();
    }

    public double getAngleFromBase() {
        return arm.getAngle() - BASE_ANGLE;
    }
}
