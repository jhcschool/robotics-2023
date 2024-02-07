package org.firstinspires.ftc.teamcode.autonomous.far;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;

public class RedTrajectoryRepo implements TrajectoryRepo {
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(-36, -72 + RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(90));
    private Pose2d floorPlacePose;
    private Pose2d backdropPose;
    private static final Pose2d PARK_POSE = new Pose2d(60, -12, Math.toRadians(180));

    public RedTrajectoryRepo(MecanumDrive drive) {
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
                floorPlacePose = new Pose2d(-47.5 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, -24.75 - RobotConstraints.CLAW_LENGTH_FROM_CENTER, Math.toRadians(90));
                break;
            }
            case CENTER: {
                floorPlacePose = new Pose2d(-36 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, -24.75 - RobotConstraints.CLAW_LENGTH_FROM_CENTER, Math.toRadians(90));
                break;
            }
            case RIGHT: {
                floorPlacePose = new Pose2d(-24.75 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, -24.75 - RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .splineToSplineHeading(FieldInfo.getRealPose(floorPlacePose), floorPlacePose.heading)
                .build();
    }

    @Override
    public Action toWaitPosition() {
        return null;
    }

    @Override
    public Action toBackdrop(PropLocation propLocation) {
        switch (propLocation) {
            case LEFT: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, -28.5, Math.toRadians(180));
                break;
            }
            case CENTER: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, -36, Math.toRadians(180));
                break;
            }
            case RIGHT: {
                backdropPose = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, -42.5, Math.toRadians(180));
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
                .strafeToLinearHeading(FieldInfo.getRealVector(new Vector2d(36, -12)), Math.toRadians(180))
                .strafeToLinearHeading(FieldInfo.getRealVector(PARK_POSE.position), PARK_POSE.heading)
                .build();

    }
}
