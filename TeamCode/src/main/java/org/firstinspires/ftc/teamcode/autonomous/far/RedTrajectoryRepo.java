package org.firstinspires.ftc.teamcode.autonomous.far;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;

public class RedTrajectoryRepo implements TrajectoryRepo {
    private MecanumDrive drive;
    private PropLocation propLocation;
    private static final Pose2d START_POSE = new Pose2d(-36, -71.6 + RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(90));
    private Pose2d floorPlacePose;
    private static final Pose2d WAIT_POSE = new Pose2d(-48, -60, Math.toRadians(180));
    private static final Pose2d INTERMEDIATE_POSE = new Pose2d(24, -60, Math.toRadians(180));
    private Pose2d backdropPose;
    private static final Pose2d CENTER_PARK_POSE = new Pose2d(60, -12, Math.toRadians(180));
    private static final Pose2d INNER_PARK_POSE = new Pose2d(64, -62, Math.toRadians(180));

    public RedTrajectoryRepo(MecanumDrive drive, PropLocation propLocation) {
        this.drive = drive;
        this.propLocation = propLocation;

        switch (propLocation) {
            case LEFT: {
                floorPlacePose = new Pose2d(-47.75 + RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, -26 - RobotConstraints.CLAW_LENGTH_FROM_CENTER, Math.toRadians(90));
                break;
            }
            case CENTER: {
                floorPlacePose = new Pose2d(-36, -24.25 - RobotConstraints.CLAW_LENGTH_FROM_CENTER, Math.toRadians(90));
                break;
            }
            case RIGHT: {
                floorPlacePose = new Pose2d(-24.25 - RobotConstraints.CLAW_LENGTH_FROM_CENTER, -30 - RobotConstraints.LEFT_CLAW_WIDTH_FROM_CENTER, Math.toRadians(0));
                break;
            }
        }

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
    }

    @Override
    public Pose2d getStartPose() {
        return FieldInfo.getRealPose(START_POSE);
    }

    @Override
    public Action toFloorPlace() {
        return drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .splineToSplineHeading(FieldInfo.getRealPose(floorPlacePose), floorPlacePose.heading)
                .build();
    }

    @Override
    public Action toWaitPosition() {
        return drive.actionBuilder(FieldInfo.getRealPose(floorPlacePose))
                .strafeToLinearHeading(FieldInfo.getRealVector(WAIT_POSE.position), WAIT_POSE.heading)
                .build();
    }

    @Override
    public Action toIntermediatePosition() {
        return drive.actionBuilder(FieldInfo.getRealPose(WAIT_POSE))
                .strafeToLinearHeading(FieldInfo.getRealVector(INTERMEDIATE_POSE.position), INTERMEDIATE_POSE.heading)
                .build();
    }

    @Override
    public Action toBackdrop() {
        return drive.actionBuilder(FieldInfo.getRealPose(INTERMEDIATE_POSE))
                .strafeToLinearHeading(FieldInfo.getRealVector(backdropPose.position), backdropPose.heading)
                .build();
    }

    @Override
    public Action toPark(ParkingLocation parkingLocation) {
        switch (parkingLocation) {
            case CENTER:
                return drive.actionBuilder(FieldInfo.getRealPose(backdropPose))
                        .strafeToLinearHeading(FieldInfo.getRealVector(new Vector2d(36, -12)), Math.toRadians(180))
                        .strafeToLinearHeading(FieldInfo.getRealVector(CENTER_PARK_POSE.position), CENTER_PARK_POSE.heading)
                        .build();
            case INNER:
                return drive.actionBuilder(FieldInfo.getRealPose(INTERMEDIATE_POSE))
//                        .strafeToLinearHeading(FieldInfo.getRealVector(new Vector2d(36, -60)), Math.toRadians(180))
                        .strafeToLinearHeading(FieldInfo.getRealVector(INNER_PARK_POSE.position), INNER_PARK_POSE.heading)
                        .build();
        }

        return null;

    }
}
