package org.firstinspires.ftc.teamcode.autonomous.far;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.game.PropLocation;

public interface TrajectoryRepo {

    Pose2d getStartPose();

    Action toFloorPlace(PropLocation propLocation);

    Action toWaitPosition();

    Action toBackdrop(PropLocation propLocation);

    Action toPark();
}
