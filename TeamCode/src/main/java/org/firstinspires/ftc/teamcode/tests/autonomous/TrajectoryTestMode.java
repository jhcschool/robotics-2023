package org.firstinspires.ftc.teamcode.tests.autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.FieldInfo;
import org.firstinspires.ftc.teamcode.robot.RobotConstraints;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Trajectory Test", group = "Autonomous Tests")
public class TrajectoryTestMode extends Mode {
    enum State {
        BEFORE_FLOOR_PLACE,
        NAVIGATE_TO_FLOOR_PLACE,
        NAVIGATE_TO_BACKSTAGE,
    }

    enum TapeLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    private static final Pose2d START_POSE = new Pose2d(12, -72 + RobotConstraints.LENGTH_FROM_CENTER, Math.toRadians(90));
    private static final Pose2d INTERMITTENT_POSE = new Pose2d(12, -36, Math.toRadians(90));
    private static final Pose2d BACKSTAGE_POSE = new Pose2d(60 - RobotConstraints.LENGTH_FROM_CENTER, -36, Math.toRadians(180));

    private MecanumDrive drive;
    private Action beforeFloorPlace;
    private Action navigateToFloorPlace;
    private Action navigateToBackstage;

    private State state = State.BEFORE_FLOOR_PLACE;
    private TapeLocation tapeLocation = TapeLocation.CENTER;

    private SequentialAction actionSequence;

    @Override
    public void onInit() {
        super.onInit();

        drive = new MecanumDrive(hardwareMap, FieldInfo.getRealPose(START_POSE));

        beforeFloorPlace = drive.actionBuilder(FieldInfo.getRealPose(START_POSE))
                .strafeToLinearHeading(FieldInfo.getRealVector(INTERMITTENT_POSE.position), INTERMITTENT_POSE.heading)
                .build();

        TrajectoryActionBuilder navigateToFloorPlaceBuilder = drive.actionBuilder(FieldInfo.getRealPose(INTERMITTENT_POSE));

        Pose2d lastPose;

        switch (tapeLocation) {
            case LEFT: {
                navigateToFloorPlaceBuilder.turnTo(Math.toRadians(135));

                Vector2d leftTapePosition = new Vector2d(0.6, -24.8);
                Vector2d unitVector = leftTapePosition.div(leftTapePosition.norm());

                Vector2d endPosition = leftTapePosition.minus(unitVector.times(RobotConstraints.CLAW_LENGTH_FROM_CENTER));

                navigateToFloorPlaceBuilder.strafeToLinearHeading(FieldInfo.getRealVector(endPosition), Math.toRadians(135));

                lastPose = FieldInfo.getRealPose(new Pose2d(leftTapePosition, Math.toRadians(135)));

                break;
            }
            case CENTER: {
                Vector2d centerTapePosition = new Vector2d(12, -24);
                Vector2d unitVector = centerTapePosition.div(centerTapePosition.norm());

                Vector2d endPosition = centerTapePosition.minus(unitVector.times(RobotConstraints.CLAW_LENGTH_FROM_CENTER));

                navigateToFloorPlaceBuilder.strafeToLinearHeading(FieldInfo.getRealVector(endPosition), Math.toRadians(90));

                lastPose = FieldInfo.getRealPose(new Pose2d(centerTapePosition, Math.toRadians(90)));

                break;
            }
            case RIGHT:
                navigateToFloorPlaceBuilder.turnTo(Math.toRadians(45));

                Vector2d rightTapePosition = new Vector2d(23.4, -24.8);
                Vector2d unitVector = rightTapePosition.div(rightTapePosition.norm());

                Vector2d endPosition = rightTapePosition.minus(unitVector.times(RobotConstraints.CLAW_LENGTH_FROM_CENTER));

                navigateToFloorPlaceBuilder.strafeToLinearHeading(FieldInfo.getRealVector(endPosition), Math.toRadians(45));

                lastPose = FieldInfo.getRealPose(new Pose2d(rightTapePosition, Math.toRadians(45)));

                break;

            default:
                throw new IllegalStateException("Unexpected value: " + tapeLocation);
        }

        navigateToFloorPlace = navigateToFloorPlaceBuilder.build();

        navigateToBackstage = drive.actionBuilder(lastPose)
                .strafeToLinearHeading(FieldInfo.getRealVector(BACKSTAGE_POSE.position), BACKSTAGE_POSE.heading)
                .build();

        actionSequence = new SequentialAction(beforeFloorPlace, navigateToFloorPlace, navigateToBackstage);
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        drive.updatePoseEstimate();

        actionSequence.run(telemetryPacket);
    }
}
