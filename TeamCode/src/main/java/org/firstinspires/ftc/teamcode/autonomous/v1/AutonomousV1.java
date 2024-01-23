package org.firstinspires.ftc.teamcode.autonomous.v1;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.DelayedAction;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.AllianceColor;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;
import org.firstinspires.ftc.teamcode.vision.TeamPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Autonomous V1", group = "Autonomous")
public class AutonomousV1 extends Mode {

    private AllianceColor allianceColor = AllianceColor.RED;
    private boolean confirmedAlliance = false;
    private GrizzlyGamepad gamepad;

    private MecanumDrive drive;
    private TrajectoryRepo trajectoryRepo;
    private VisionSystem visionSystem;
    private ArmSystem armSystem;
    private WristSystem wristSystem;
    private ClawSystem clawSystem;
    private Action action;

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        visionSystem = new VisionSystem();
        armSystem = new ArmSystem(hardwareMap);
        wristSystem = new WristSystem(hardwareMap);
        clawSystem = new ClawSystem(hardwareMap);
    }

    @Override
    public void beforeStartLoop() {
        super.beforeStartLoop();

        if (!confirmedAlliance) {
            telemetry.addData("Current Alliance", allianceColor.toString());
            telemetry.addLine("Press A for red alliance, B for blue alliance, and X to confirm");

            if (gamepad.getButton(Button.A)) {
                allianceColor = AllianceColor.RED;
            } else if (gamepad.getButton(Button.B)) {
                allianceColor = AllianceColor.BLUE;
            }

            if (gamepad.getButton(Button.X)) {
                telemetry.clearAll();
                telemetry.addData("Alliance", allianceColor.toString());

                confirmedAlliance = true;
                initForAlliance();
            }
        }

        drive.updatePoseEstimate();
        visionSystem.update();
    }

    public void initForAlliance() {
        switch (allianceColor) {
            case RED:
                trajectoryRepo = new RedTrajectoryRepo(drive);
                break;
            case BLUE:
                trajectoryRepo = new BlueTrajectoryRepo(drive);
                break;
        }

        drive.pose = trajectoryRepo.getStartPose();

        visionSystem.initForAlliance(hardwareMap, allianceColor);
    }

    @Override
    public void onStart() {
        super.onStart();

        if (!confirmedAlliance) {
            initForAlliance(); // Shouldn't happen, but just in case
            return;
        }

        visionSystem.stopStreaming();

        Action clawClose = new ParallelAction(
                clawSystem.closeLeft(),
                clawSystem.closeRight()
        );

        Action toFloorPlace = new SequentialAction(
                trajectoryRepo.toFloorPlace(visionSystem.getPropLocation()),
                clawSystem.openLeft()
        );

        Action toFirstBackdrop = new SequentialAction(
                new ParallelAction(
                        trajectoryRepo.toFirstBackdrop(visionSystem.getPropLocation()),
                        new DelayedAction(0.3, armSystem.raiseArm())
                ),
                clawSystem.openRight());

        action = new SequentialAction(
                clawClose,
                toFloorPlace,
                toFirstBackdrop
        );
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        drive.updatePoseEstimate();
        action.run(packet);

        double angle = armSystem.getAngle();
        wristSystem.update(angle);

        telemetry.addData("Pose", drive.pose);
        telemetry.addData("Prop Location", visionSystem.getPropLocation());

        double currentTime = timer.seconds();
        telemetry.addData("Loop Time", currentTime - lastTime);
        lastTime = currentTime;

        telemetry.addData("Arm Angle", Math.toDegrees(angle));
    }

}
