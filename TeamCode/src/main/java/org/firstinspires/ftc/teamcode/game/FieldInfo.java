package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldInfo {

    public static final double SIZE_RATIO = 141.75 / 144.0; // Real field size divided by projected field size
    public static final double OUTSIDE_CUTOFF = 0.0; // Outside x inches of field are cut off

    public static Pose2d getRealPose(Pose2d pose) {
        return new Pose2d(getRealVector(pose.position), pose.heading);
    }

    private static double fixAxis(double value) {
        if (value > (72.0 - OUTSIDE_CUTOFF)) {
            return (72.0 - OUTSIDE_CUTOFF);
        } else if (value < (-72 + OUTSIDE_CUTOFF)) {
            return (-72.0 + OUTSIDE_CUTOFF);
        }

        return value;
    }

    public static Pose2d getProjectedPose(Pose2d pose) {
        return new Pose2d(pose.position.div(SIZE_RATIO), pose.heading);
    }

    public static double getRealDistance(double distance) {
        return distance * SIZE_RATIO;
    }

    public static double getProjectedDistance(double distance) {
        return distance / SIZE_RATIO;
    }

    public static Vector2d getRealVector(Vector2d vector) {
        Vector2d realVector = vector.times(SIZE_RATIO);
        // Cut off outside of field
        return new Vector2d(fixAxis(realVector.x), fixAxis(realVector.y));
    }

    public static Vector2d getProjectedVector(Vector2d vector) {
        return vector.div(SIZE_RATIO);
    }
}
