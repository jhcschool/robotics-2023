package org.firstinspires.ftc.teamcode.robot;

public class Potentiometer {
    public static double getArmRotDegrees(double voltage) {
        double degrees = 270.0;
        if (voltage > 0.0) // make sure it's not zero
        {
            // formula derived from the following formula from potentiometer's data sheet on REV's website
            // voltage = 445.5 * (degrees - 270.0) / (degrees * (degrees - 270.0) - 36450.0)
            degrees = 6.75 * (-Math.sqrt(1089.0 / (voltage * voltage) - (1320.0 / voltage) + 1200.0) + (33.0 / voltage) + 20.0);
        }
        return degrees;
    }
}
