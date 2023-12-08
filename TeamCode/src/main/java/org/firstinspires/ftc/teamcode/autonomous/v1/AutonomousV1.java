package org.firstinspires.ftc.teamcode.autonomous.v1;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private Action action;

    public AutonomousV1() {
        super(LynxModule.BulkCachingMode.AUTO); // TODO: change to MANUAL
    }

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        visionSystem = new VisionSystem();
        armSystem = new ArmSystem(hardwareMap);
        wristSystem = new WristSystem(hardwareMap);
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

            if (!gamepad.getButton(Button.X)) {
                return;
            }

            telemetry.clearAll();
            telemetry.addData("Alliance", allianceColor.toString());

            confirmedAlliance = true;
            initForAlliance();
        }

        drive.updatePoseEstimate();
        visionSystem.update();
    }

    void initForAlliance() {
        switch (allianceColor) {
            case RED:
                trajectoryRepo = new RedTrajectoryRepo(drive);
                break;
            case BLUE:
                break;
        }

        drive.pose = trajectoryRepo.getStartPose();

        visionSystem.initForAlliance(hardwareMap, allianceColor);
    }

    @Override
    public void onStart() {
        super.onStart();

        visionSystem.stopStreaming();

        Action toFirstBackdrop = new SequentialAction(
                trajectoryRepo.toFirstBackdrop(visionSystem.getPropLocation()),
                armSystem.raiseArm()
        );

        Action toFirstPixelStack = new SequentialAction(
                trajectoryRepo.toFirstPixelStack(),
                armSystem.lowerArm()
        );

        Action toCycleBackdrop = new SequentialAction(
                trajectoryRepo.toCycleBackdrop(),
                armSystem.raiseArm()
        );

        action = new SequentialAction(
                trajectoryRepo.toFloorPlace(visionSystem.getPropLocation()),
                toFirstBackdrop,
                toFirstPixelStack,
                toCycleBackdrop
        );
    }

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        drive.updatePoseEstimate();
        action.run(packet);
        wristSystem.update(armSystem.getAngleFromBase());

        telemetry.addData("Pose", drive.pose);
        telemetry.addData("Prop Location", visionSystem.getPropLocation());

        double currentTime = timer.seconds();
        telemetry.addData("Loop Time", currentTime - lastTime);
        lastTime = currentTime;
    }

}
