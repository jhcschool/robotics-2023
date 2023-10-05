package org.firstinspires.ftc.teamcode.tests.controlled;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@TeleOp(name = "Team Prop Test", group = "Controlled Tests")
public class TeamPropTestMode extends Mode {

    private static final Scalar RED_LOWER_THRESHOLD = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER_THRESHOLD2 = new Scalar(160, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD2 = new Scalar(179, 255, 255);

    private static final Scalar BLUE_LOWER_THRESHOLD = new Scalar(90, 100, 100);
    private static final Scalar BLUE_UPPER_THRESHOLD = new Scalar(120, 255, 255);

    private TeamPropProcessor teamPropProcessor;
    private VisionPortal visionPortal;

    @Override
    public void onInit() {
        super.onInit();

        teamPropProcessor = new TeamPropProcessor.Builder()
//                .setColor(RED_LOWER_THRESHOLD, RED_UPPER_THRESHOLD, RED_LOWER_THRESHOLD2, RED_UPPER_THRESHOLD2)
                .setColor(BLUE_LOWER_THRESHOLD, BLUE_UPPER_THRESHOLD)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, HardwareID.CAMERA))
                .addProcessor(teamPropProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        telemetry.addData("Camera FPS", visionPortal.getFps());
        telemetry.addData("Detection", teamPropProcessor.getDetection());
    }


}
