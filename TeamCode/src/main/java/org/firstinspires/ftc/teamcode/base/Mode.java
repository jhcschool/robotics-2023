package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Disabled
public class Mode extends LinearOpMode {

    private final Object beforeStartLoopSync = new Object();
    private Thread beforeStartLoopThread;
    private boolean isBeforeStart = true;
    private LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.AUTO;
    private List<LynxModule> allHubs;

    public Mode() {
        super();
    }

    public Mode(LynxModule.BulkCachingMode bulkCachingMode) {
        super();
        this.bulkCachingMode = bulkCachingMode;
    }

    @Override
    public final void runOpMode() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(bulkCachingMode);
        }

        onInit();
        beforeStart();
        if (isStopRequested()) {
            return;
        }
        onStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            tick(packet);
            dashboard.sendTelemetryPacket(packet);
        }
        onEnd();
    }

    public void onInit() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void beforeStart() {
        beforeStartLoopThread = new Thread(() -> {
            while (true) {
                synchronized (beforeStartLoopSync) {
                    if (!isBeforeStart) {
                        break;
                    }
                }
                beforeStartLoop();
            }
        });
        beforeStartLoopThread.start();
        waitForStart();
        synchronized (beforeStartLoopSync) {
            isBeforeStart = false;
        }
        try {
            beforeStartLoopThread.join(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void beforeStartLoop() {
        telemetry.update();
    }


    public void onStart() {
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void tick(TelemetryPacket telemetryPacket) {
        telemetry.update();
    }

    public void onEnd() {
        telemetry.addData("Status", "Ended");
        telemetry.update();
    }

    protected void clearBulkCache() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
