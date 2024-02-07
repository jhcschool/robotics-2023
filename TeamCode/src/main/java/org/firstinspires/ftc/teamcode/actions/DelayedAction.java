package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayedAction implements Action {
    public DelayedAction(double delay, Action action) {
        this.delay = delay;
        this.action = action;
    }

    public DelayedAction(double delay) {
        this.delay = delay;
    }

    private double delay;
    private Action action;
    private ElapsedTime elapsedTime;


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (elapsedTime == null) {
            elapsedTime = new ElapsedTime();
        }

        if (elapsedTime.seconds() > delay) {
            if (action != null) {
                return action.run(telemetryPacket);
            }

            return false;
        }

        return true;
    }
}
