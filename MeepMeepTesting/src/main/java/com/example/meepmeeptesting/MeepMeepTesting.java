package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.awt.Robot;

public class MeepMeepTesting {


    public static final double FIELD_RATIO = 141.1 / 144.0; // Real field size divided by projected field size

    enum TapePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(RobotConstraints.WIDTH, RobotConstraints.LENGTH)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15).build();

        TapePosition tapePosition = TapePosition.CENTER;

        Pose2d startPose = new Pose2d(12, -72 + RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(90));
        Pose2d intermittentPosition = new Pose2d(12, -36, Math.toRadians(90));


        double distanceForward = 0.0;
        switch (tapePosition) {
            case LEFT:
                Vector2d leftTapePosition = new Vector2d(0.6, -24.8);
                distanceForward = leftTapePosition.minus(intermittentPosition.vec()).norm() - RobotConstraints.CLAW_LENGTH_FROM_CENTER;
                break;
            case CENTER:
                Vector2d centerTapePosition = new Vector2d(12, -24);
                distanceForward = centerTapePosition.minus(intermittentPosition.vec()).norm() - RobotConstraints.CLAW_LENGTH_FROM_CENTER;
                break;
            case RIGHT:
                Vector2d rightTapePosition = new Vector2d(23.4, -24.8);
                distanceForward = rightTapePosition.minus(intermittentPosition.vec()).norm() - RobotConstraints.CLAW_LENGTH_FROM_CENTER;
                break;
        }

//        double rightTapeRotationDelta = Math.toRadians(-45);

//        Vector2d centerTapePosition = new Vector2d(12, -24);
//        double centerDistanceForward = centerTapePosition.minus(intermittentPosition.vec()).norm() - RobotConstraints.CLAW_LENGTH_FROM_CENTER;

//        Pose2d leftTapePose = new Pose2d(0.0, -24, Math.toRadians(135));
//        Vector2d leftTapePoseCenter = leftTapePose.headingVec().times(RobotConstraints.CLAW_LENGTH_FROM_CENTER);
//        leftTapePose = new Pose2d(leftTapePose.vec().minus(leftTapePoseCenter), leftTapePose.getHeading());

        Pose2d backStagePosition = new Pose2d(60 - RobotConstraints.OUTPUT_LENGTH_FROM_CENTER, -36, Math.toRadians(-180));

        Pose2d stackPosition = new Pose2d(-70 + RobotConstraints.CLAW_LENGTH_FROM_CENTER, -36, Math.toRadians(-180));

        TrajectorySequenceBuilder builder = bot.getDrive().trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(projectedPoseToRealPose(intermittentPosition));

        switch (tapePosition) {
            case LEFT:
                builder.turn(Math.toRadians(45));
                break;
            case CENTER:
                break;
            case RIGHT:
                builder.turn(Math.toRadians(-45));
                break;
        }

        builder.forward(distanceForward);

        builder.lineToLinearHeading(projectedPoseToRealPose(backStagePosition))
                .lineToLinearHeading(projectedPoseToRealPose(stackPosition))
                .build();

        TrajectorySequence sequence = builder.build();

        bot.followTrajectorySequence(sequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }

    private static Pose2d projectedPoseToRealPose(Pose2d pose) {
        return new Pose2d(pose.vec().times(FIELD_RATIO), pose.getHeading());
    }
}