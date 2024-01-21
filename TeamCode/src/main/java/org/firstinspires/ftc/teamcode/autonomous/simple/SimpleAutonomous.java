package org.firstinspires.ftc.teamcode.autonomous.simple;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.AllianceColor;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Simple Autonomous", group = "Autonomous")
public class SimpleAutonomous extends Mode {
    private MecanumDrive drive;
    private Action action;
    private Claw claw;

    @Override
    public void onInit() {
        super.onInit();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        claw = new Claw(hardwareMap);
    }

    @Override
    public void beforeStartLoop() {
        super.beforeStartLoop();
    }

    @Override
    public void onStart() {
        super.onStart();

        action = drive.actionBuilder(new Pose2d(0, 0, 0)).lineToX(40).build();
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);
        drive.updatePoseEstimate();

        if (action.run(packet)) {
            claw.closeLeft();
            claw.closeRight();
        } else {
            claw.openRight();
            claw.openLeft();
        }

        telemetry.addData("Pose", drive.pose);

        double currentTime = timer.seconds();
        telemetry.addData("Loop Time", currentTime - lastTime);
        lastTime = currentTime;
    }

}
