package org.firstinspires.ftc.teamcode.autonomous.v1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;

public class BlueTrajectoryRepo implements TrajectoryRepo {
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(12, 72 - RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(270));
    private Pose2d floorPlacePose;
    private static final Pose2d CENTER_BACKDROP_POSE = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, 36 + RobotConstraints.RIGHT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
    private Pose2d firstBackdropPose = CENTER_BACKDROP_POSE;
    private static final Pose2d PIXEL_STACK_POSE = new Pose2d(-70 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 36, Math.toRadians(180));

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
            case LEFT: {
                floorPlacePose = new Pose2d(0.75 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
            case CENTER: {
                floorPlacePose = new Pose2d(12 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
            case RIGHT: {
                floorPlacePose = new Pose2d(23.25 + RobotConstraints.RIGHT_CLAW_WIDTH_FROM_CENTER, 24.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .strafeToLinearHeading(FieldInfo.getRealPose(floorPlacePose).position, floorPlacePose.heading)
                .build();
    }

    @Override
    public Action toFirstBackdrop(PropLocation propLocation) {
        switch (propLocation) {
            case LEFT: {
                firstBackdropPose = new Pose2d(firstBackdropPose.position.x, 29.5 + RobotConstraints.RIGHT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
            case CENTER: {
                break;
            }
            case RIGHT: {
                firstBackdropPose = new Pose2d(firstBackdropPose.position.x, 42.5 + RobotConstraints.RIGHT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(floorPlacePose))
                .strafeToLinearHeading(FieldInfo.getRealVector(firstBackdropPose.position), firstBackdropPose.heading)
                .build();
    }

    @Override
    public Action toFirstPixelStack() {
        return drive.actionBuilder(FieldInfo.getRealPose(firstBackdropPose))
                .strafeToLinearHeading(FieldInfo.getRealVector(PIXEL_STACK_POSE.position), PIXEL_STACK_POSE.heading)
                .build();
    }

    @Override
    public Action toCycleBackdrop() {
        return drive.actionBuilder(FieldInfo.getRealPose(PIXEL_STACK_POSE))
                .strafeToLinearHeading(FieldInfo.getRealVector(CENTER_BACKDROP_POSE.position), CENTER_BACKDROP_POSE.heading)
                .build();
    }

    @Override
    public Action toPixelStack() {
        return drive.actionBuilder(FieldInfo.getRealPose(PIXEL_STACK_POSE))
                .strafeToLinearHeading(FieldInfo.getRealVector(PIXEL_STACK_POSE.position), PIXEL_STACK_POSE.heading)
                .build();
    }
}
