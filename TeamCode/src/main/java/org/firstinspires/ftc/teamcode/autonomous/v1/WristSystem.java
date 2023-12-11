package org.firstinspires.ftc.teamcode.autonomous.v1;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrist.Wrist;
import org.firstinspires.ftc.teamcode.wrist.WristController;

public class WristSystem {
    private Wrist wrist;
    private WristController wristController;

    public WristSystem(HardwareMap hardwareMap) {
        wrist = new Wrist(hardwareMap);
        wristController = new WristController(wrist);
    }

    public void update(double armAngleFromBase) {
        wristController.update(armAngleFromBase);
    }
}
