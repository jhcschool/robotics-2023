package org.firstinspires.ftc.teamcode.autonomous.v1;

import android.util.Size;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.AllianceColor;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.robot.HardwareID;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

public class VisionSystem {

    private static final Scalar RED_LOWER_THRESHOLD = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER_THRESHOLD2 = new Scalar(160, 100, 100);
    private static final Scalar RED_UPPER_THRESHOLD2 = new Scalar(179, 255, 255);

    private static final Scalar BLUE_LOWER_THRESHOLD = new Scalar(90, 100, 100);
    private static final Scalar BLUE_UPPER_THRESHOLD = new Scalar(120, 255, 255);

    private TeamPropProcessor teamPropProcessor;
    private VisionPortal visionPortal;

    private PropLocation[] lastDetections = new PropLocation[10];


    public VisionSystem() {
    }

    public void initForAlliance(HardwareMap hardwareMap, AllianceColor allianceColor) {
//        TeamPropProcessor.Builder builder = new TeamPropProcessor.Builder();
//
//        switch (allianceColor) {
//            case RED:
//                builder.setColor(RED_LOWER_THRESHOLD, RED_UPPER_THRESHOLD, RED_LOWER_THRESHOLD2, RED_UPPER_THRESHOLD2);
//                break;
//            case BLUE:
//                builder.setColor(BLUE_LOWER_THRESHOLD, BLUE_UPPER_THRESHOLD);
//                break;
//        }
//
//        teamPropProcessor = builder.build();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, HardwareID.CAMERA))
//                .addProcessor(teamPropProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
    }

    public void update() {
//        PropLocation detection = teamPropProcessor.getDetection();
//
//        if (detection != null) {
//            for (int i = lastDetections.length - 1; i > 0; i--) {
//                lastDetections[i] = lastDetections[i - 1];
//            }
//
//            lastDetections[0] = detection;
//        }
    }

    public void stopStreaming() {

//        visionPortal.stopStreaming();
    }

    public PropLocation getPropLocation() {
//        // Return most common prop location
//
//        int[] counts = new int[PropLocation.values().length];
//
//        for (PropLocation detection : lastDetections) {
//            if (detection != null) {
//                counts[detection.ordinal()]++;
//            }
//        }
//
//        int maxCount = 0;
//        PropLocation maxPropLocation = PropLocation.CENTER;
//
//        for (PropLocation propLocation : PropLocation.values()) {
//            if (counts[propLocation.ordinal()] > maxCount) {
//                maxCount = counts[propLocation.ordinal()];
//                maxPropLocation = propLocation;
//            }
//        }
//
//        return maxPropLocation;

        return PropLocation.LEFT;
    }

}
