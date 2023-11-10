package org.firstinspires.ftc.teamcode.arm;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.PIDFController;

@Config
public class ArmController {

    private static PIDFController.PIDCoefficients PID_COEFFICIENTS = new PIDFController.PIDCoefficients(0.0, 0.0, 0.0);
    private static double FEEDFORWARD_COEFFICIENT = 1.0;

    private PIDFController pidfController;

    public ArmController() {
        pidfController = new PIDFController(PID_COEFFICIENTS, ArmController::feedforward);
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
        return pidfController.update(currentAngle);
    }

    private static double feedforward(double angle, @Nullable Double velocity) {
        return FEEDFORWARD_COEFFICIENT * Math.cos(angle);
    }
}
