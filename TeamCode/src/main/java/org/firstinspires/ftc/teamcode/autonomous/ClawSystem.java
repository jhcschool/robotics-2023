package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.claw.Claw;

public class ClawSystem {
    private static final double TIME_TO_MOVE = 0.4;

    private Claw claw;

    public ClawSystem(HardwareMap hardwareMap) {
        claw = new Claw(hardwareMap);
    }

    private class ClawAction implements Action {
        private ElapsedTime timeSinceClip;
        private boolean left;
        private boolean closed;

        public ClawAction(boolean left, boolean closed) {
            this.left = left;
            this.closed = closed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timeSinceClip == null) {
                timeSinceClip = new ElapsedTime();
            }

            if (timeSinceClip.seconds() > TIME_TO_MOVE) {
                return false;
            }

            if (closed) {
                if (left) {
                    claw.closeLeft();
                } else {
                    claw.closeRight();
                }
            } else {
                if (left) {
                    claw.openLeft();
                } else {
                    claw.openRight();
                }
            }

            return true;
        }
    }

    public Action closeLeft() {
        return new ClawAction(true, true);
    }

    public Action closeRight() {
        return new ClawAction(false, true);
    }

    public Action openLeft() {
        return new ClawAction(true, false);
    }

    public Action openRight() {
        return new ClawAction(false, false);
    }

}
