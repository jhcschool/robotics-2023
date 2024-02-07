package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
public class ArmController {

    // Currently working: K_P = 0.4, power multiplier = 1.0, deadzone = 0, max_power = 1.0
    public static double K_P = 0.55;
    public static double K_I = 0.0;
    public static double K_D = 0.0;
    public static double K_STATIC = 0.15;
    public static double K_COS = 0.1;
    public static double MAX_POWER = 1.0;
    public static double POWER_MULTIPLIER = 1.0;
    public static double DEADZONE = 10;
    public static double ANGLE_WEIGHTING = 0.6;

    public static double POSITION_TOLERANCE = Math.toRadians(5.0);
    public static double VELOCITY_TOLERANCE = Math.toRadians(5.0);

    private PIDController pidController;

    public ArmController() {
        pidController = new PIDController(K_P, K_I, K_D);
        pidController.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }

    public void setTargetAngle(double targetAngle) {
        pidController.setSetPoint(targetAngle);

        pidController.clearTotalError();
    }

    public double update(double currentAngle) {
        // For tuning with dashboard
        pidController.setPID(K_P, K_I, K_D);

        if (Math.abs(currentAngle - Math.toRadians(90)) < Math.toRadians(DEADZONE)) {
            return 0.0;
        }

        double power = pidController.calculate(currentAngle) * POWER_MULTIPLIER;
        power += Math.signum(power) * K_STATIC;

        power -= power * ANGLE_WEIGHTING * Math.abs(Math.sin(currentAngle));

        power += +feedforward(currentAngle);
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
    }

    public double getTargetAngle() {
        return pidController.getSetPoint();
    }

    public boolean atTarget() {
        return pidController.atSetPoint();
    }

    public void setPositionTolerance(double positionTolerance) {
        pidController.setTolerance(positionTolerance, pidController.getTolerance()[1]);
    }

    public void setVelocityTolerance(double velocityTolerance) {
        pidController.setTolerance(pidController.getTolerance()[0], velocityTolerance);
    }

    private static double feedforward(double angle) {
        return K_COS * Math.cos(angle);
    }
}
