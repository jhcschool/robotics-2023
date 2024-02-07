package org.firstinspires.ftc.teamcode.autonomous.far;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;

public class BlueTrajectoryRepo implements TrajectoryRepo {
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(12, 72 - RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(270));
    private Pose2d floorPlacePose;
    private Pose2d backdropPose;
    private static final Pose2d PARK_POSE = new Pose2d(60, 12, Math.toRadians(180));

    public BlueTrajectoryRepo(MecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public Pose2d getStartPose() {
        return FieldInfo.getRealPose(START_POSE);
    }

    @Override
    public Action toFloorPlace(PropLocation propLocation) {
        switch (propLocation) {
            case RIGHT: {
                floorPlacePose = new Pose2d(0.75 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
            case CENTER: {
                floorPlacePose = new Pose2d(12 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
            case LEFT: {
                floorPlacePose = new Pose2d(23.25 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .strafeToLinearHeading(FieldInfo.getRealPose(floorPlacePose).position, floorPlacePose.heading)
                .build();
    }

    @Override
    public Action toBackdrop(PropLocation propLocation) {
        switch (propLocation) {
            case RIGHT: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, 29.5, Math.toRadians(180));
                break;
            }
            case CENTER: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, 36, Math.toRadians(180));
                break;
            }
            case LEFT: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, 43.5, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(floorPlacePose))
                .strafeToLinearHeading(FieldInfo.getRealVector(backdropPose.position), backdropPose.heading)
                .build();
    }

    @Override
    public Action toPark() {
        return drive.actionBuilder(FieldInfo.getRealPose(backdropPose))
                .strafeToLinearHeading(FieldInfo.getRealVector(new Vector2d(36, 12)), Math.toRadians(180))
                .strafeToLinearHeading(FieldInfo.getRealVector(PARK_POSE.position), PARK_POSE.heading)
                .build();
    }
}
