package org.firstinspires.ftc.teamcode.autonomous.v1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.wrist.WristController;

@Config
public class ArmSystem {
    public static double RAISED_ANGLE = Math.toRadians(130);
    private static final double BASE_ANGLE = -WristController.ARM_ANGLE_BASE_TO_LEVEL;
    public static double ERROR_THRESHOLD = Math.toRadians(10);
    public static double OSCILLATION_TIME = 0.2;
    public static double POWER_MULTIPLIER = 0.7;
    public static double MAX_POWER = 0.7;

    private Arm arm;

    public ArmSystem(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
    }

    private class ArmAction implements Action {
        private ArmController armController;
        private ElapsedTime timeSinceHit = null;

        public ArmAction(double targetAngle) {
            armController = new ArmController();
            armController.setTargetAngle(targetAngle);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timeSinceHit != null && timeSinceHit.seconds() > OSCILLATION_TIME) {
                arm.setPower(0.0);
                return false;
            }

            if (Math.abs(armController.getTargetAngle() - arm.getAngle()) < ERROR_THRESHOLD) {
                timeSinceHit = new ElapsedTime();
            }

            double power = armController.update(arm.getAngle()) * POWER_MULTIPLIER;
            power = Math.min(Math.abs(power), MAX_POWER) * Math.signum(power);

            arm.setPower(power);
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
}
