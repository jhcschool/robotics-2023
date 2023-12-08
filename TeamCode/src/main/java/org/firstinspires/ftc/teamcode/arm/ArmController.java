package org.firstinspires.ftc.teamcode.arm;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.PIDFController;

@Config
public class ArmController {

    private static PIDFController.PIDCoefficients PID_COEFFICIENTS = new PIDFController.PIDCoefficients(0.0, 0.0, 0.0);
    private static double K_STATIC = 0.0;
    private static double GRAVITY_COEFFICIENT = 0.3;

    private PIDFController pidfController;

    public ArmController() {
        pidfController = new PIDFController(PID_COEFFICIENTS, 0, 0, K_STATIC, ArmController::feedforward);
    }

    public void setTargetAngle(double targetAngle) {
        pidfController.targetPosition = targetAngle;
    }

    public void setTargetVelocity(double targetVelocity) {
        pidfController.targetVelocity = targetVelocity;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        pidfController.targetAcceleration = targetAcceleration;
    }

    public double update(double currentAngle) {
        // For tuning with dashboard
        pidfController.setPIDCoefficients(PID_COEFFICIENTS);
        pidfController.setFeedforwardCoefficients(0, 0, K_STATIC);

        return pidfController.update(currentAngle);
    }

    public double getTargetAngle() {
        return pidfController.targetPosition;
    }


    private static double feedforward(double angle, @Nullable Double velocity) {
        return GRAVITY_COEFFICIENT * Math.cos(angle);
    }
}
