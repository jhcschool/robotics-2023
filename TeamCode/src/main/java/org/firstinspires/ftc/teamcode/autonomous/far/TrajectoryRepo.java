package org.firstinspires.ftc.teamcode.autonomous.far;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.game.PropLocation;

public interface TrajectoryRepo {

    Pose2d getStartPose();

    Action toFloorPlace();

    Action toWaitPosition();

    Action toIntermediatePosition();

    Action toBackdrop();

    Action toPark(ParkingLocation parkingLocation);
}
