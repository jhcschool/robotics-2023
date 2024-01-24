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
    public static double RAISED_ANGLE = Math.toRadians(150);
    public static double OSCILLATION_TIME = 0.2;

    private Arm arm;

    public ArmSystem(HardwareMap hardwareMap) {
        arm = new Arm(hardwareMap);
    }

    public void beforeStart() {
        arm.setPower(-0.1); // Make sure it is down before start
    }

    public void onInit() {
        arm.setPower(0.0);
        arm.resetEncoder();
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

            if (armController.atTarget()) {
                timeSinceHit = new ElapsedTime();
            }

            arm.setPower(armController.update(arm.getAngle()));
            return true;
        }
    }

    public Action raiseArm() {
        return new ArmAction(RAISED_ANGLE);
    }

    public Action lowerArm() {
        return new ArmAction(Arm.BASE_ANGLE);
    }

    public Action setAngle(double angle) {
        return new ArmAction(angle);
    }

    public double getAngle() {
        return arm.getAngle();
    }
}
