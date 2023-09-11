package org.firstinspires.ftc.teamcode.tests.controlled;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Test", group = "Controlled Tests")
public class AprilTagTestMode extends Mode {

    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    @Override
    public void onInit() {
        super.onInit();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }

    @Override
    public void tick(TelemetryPacket telemetryPacket) {
        super.tick(telemetryPacket);

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            telemetry.addLine("Detection ID: " + detection.id);
            telemetry.addLine("Detection Center: " + detection.center.toString());
            telemetry.addLine("Detection Position: {" + detection.ftcPose.x + ", " + detection.ftcPose.y + ", " + detection.ftcPose.z + "}");
        }

        telemetry.addData("Camera FPS", visionPortal.getFps());
    }

}
