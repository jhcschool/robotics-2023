package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

@Config
public class WristSystem {
    private static final double FOLD_ANGLE = 210;
    private static final double TIME_TO_FOLD = 0.3;

    private Wrist wrist;
    private WristController wristController;

    public WristSystem(HardwareMap hardwareMap) {
        wrist = new Wrist(hardwareMap);
        wristController = new WristController(wrist);
    }

    private class FoldAction implements Action {
        private ElapsedTime timeSinceFold;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (timeSinceFold == null) {
                timeSinceFold = new ElapsedTime();
            }

            if (timeSinceFold.seconds() > TIME_TO_FOLD) {
                return false;
            }

            wrist.setAngle(FOLD_ANGLE);
            return true;
        }
    }

    public Action fold() {
        return new FoldAction();
    }

    public void update(double armAngle) {
        wristController.update(armAngle);
    }

    public void update(double armAngle, double armOffset) {
        wristController.update(armAngle, armOffset);
    }
}
