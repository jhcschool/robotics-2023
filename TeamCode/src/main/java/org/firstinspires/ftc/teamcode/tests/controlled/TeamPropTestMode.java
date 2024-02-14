package org.firstinspires.ftc.teamcode.tests.controlled;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

@Config
@TeleOp(name = "Team Prop Test", group = "Controlled Tests")
public class TeamPropTestMode extends Mode {

    private static final Scalar RED_LOWER_THRESHOLD = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER_THRESHOLD2 = new Scalar(160, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD2 = new Scalar(179, 255, 255);

    private static final Scalar BLUE_LOWER_THRESHOLD = new Scalar(90, 100, 100);
    private static final Scalar BLUE_UPPER_THRESHOLD = new Scalar(120, 255, 255);

    public static Point TL_LEFT = new Point(0, 300);
    public static Point BR_LEFT = new Point(150, 480);
    public static Point TL_CENTER = new Point(213, 300);
    public static Point BR_CENTER = new Point(426, 480);
    public static Point TL_RIGHT = new Point(520, 300);
    public static Point BR_RIGHT = new Point(640, 480);

    private TeamPropProcessor teamPropProcessor;
    private VisionPortal visionPortal;

    @Override
    public void onInit() {
        super.onInit();

        teamPropProcessor = new TeamPropProcessor.Builder()
//                .setColor(RED_LOWER_THRESHOLD, RED_UPPER_THRESHOLD, RED_LOWER_THRESHOLD2, RED_UPPER_THRESHOLD2)
                .setColor(BLUE_LOWER_THRESHOLD, BLUE_UPPER_THRESHOLD)
                .setLeftBox(TL_LEFT, BR_LEFT)
                .setCenterBox(TL_CENTER, BR_CENTER)
                .setRightBox(TL_RIGHT, BR_RIGHT)
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
