package org.firstinspires.ftc.teamcode.autonomous.v1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;

public class RedTrajectoryRepo implements TrajectoryRepo {
    private MecanumDrive drive;
    private static final Pose2d START_POSE = new Pose2d(12, -72 + RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(90));
    private Pose2d floorPlacePose = new Pose2d(12, -24.5 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, Math.toRadians(90));
    private static final Pose2d CENTER_BACKDROP_POSE = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, -36, Math.toRadians(180));
    private Pose2d firstBackdropPose = CENTER_BACKDROP_POSE;
    private static final Pose2d PIXEL_STACK_POSE = new Pose2d(-70 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, -36, Math.toRadians(180));

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
                floorPlacePose = new Pose2d(0.5, floorPlacePose.position.y, Math.toRadians(135));
                break;
            }
            case CENTER: {
                break;
            }
            case RIGHT: {
                floorPlacePose = new Pose2d(23.5, floorPlacePose.position.y, Math.toRadians(45));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .splineToSplineHeading(FieldInfo.getRealPose(floorPlacePose), floorPlacePose.heading)
                .build();
    }

    @Override
    public Action toFirstBackdrop(PropLocation propLocation) {
        switch (propLocation) {
            case LEFT: {
                firstBackdropPose = new Pose2d(firstBackdropPose.position.x, -29.5, Math.toRadians(180));
                break;
            }
            case CENTER: {
                break;
            }
            case RIGHT: {
                firstBackdropPose = new Pose2d(firstBackdropPose.position.x, -42.5, Math.toRadians(180));
                break;
            }
        }

        return drive.actionBuilder(FieldInfo.getRealPose(floorPlacePose))
                .strafeTo(FieldInfo.getRealVector(new Vector2d(12, -36)))
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
