package org.firstinspires.ftc.teamcode.autonomous.near;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.DelayedAction;
import org.firstinspires.ftc.teamcode.autonomous.ArmSystem;
import org.firstinspires.ftc.teamcode.autonomous.ClawSystem;
import org.firstinspires.ftc.teamcode.autonomous.VisionSystem;
import org.firstinspires.ftc.teamcode.autonomous.WristSystem;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.AllianceColor;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;

@Config
@Autonomous(name = "Near Autonomous", group = "Autonomous")
public class NearAutonomous extends Mode {

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
    private PlayStyle playStyle = PlayStyle.AGGRESSIVE;
    private ParkingLocation parkingLocation = ParkingLocation.INNER;

    @Override
    public void onInit() {
        super.onInit();

        gamepad = new GrizzlyGamepad(gamepad1);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        visionSystem = new VisionSystem();
        armSystem = new ArmSystem(hardwareMap);
        wristSystem = new WristSystem(hardwareMap);
        clawSystem = new ClawSystem(hardwareMap);

        action = new SequentialAction(
                new ParallelAction(
                        clawSystem.closeLeft(),
                        clawSystem.closeRight()
                ),
                wristSystem.fold()
        );

    }

    @Override
    public void beforeStartLoop() {
        super.beforeStartLoop();

        if (action != null && !action.run(new TelemetryPacket())) {
            action = null;
        }

        if (!confirmedAlliance) {
            telemetry.addData("Current Alliance", allianceColor.toString());
            telemetry.addLine("Press A for red alliance, B for blue alliance, and X to confirm");

            if (gamepad.getButton(Button.A)) {
                allianceColor = AllianceColor.RED;
            } else if (gamepad.getButton(Button.B)) {
                allianceColor = AllianceColor.BLUE;
            }

            telemetry.addData("Play Style", playStyle.toString());
            telemetry.addLine("Press DPad Up for aggressive play style, DPad Down for conservative play style");
            if (gamepad.getButton(Button.DPAD_UP)) {
                playStyle = PlayStyle.AGGRESSIVE;
            } else if (gamepad.getButton(Button.DPAD_DOWN)) {
                playStyle = PlayStyle.CONSERVATIVE;
            }

            telemetry.addData("Parking Location", parkingLocation.toString());
            telemetry.addLine("Press DPad left for inner parking location, DPad right for center parking location");
            if (gamepad.getButton(Button.DPAD_LEFT)) {
                parkingLocation = ParkingLocation.INNER;
            } else if (gamepad.getButton(Button.DPAD_RIGHT)) {
                parkingLocation = ParkingLocation.CENTER;
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
        armSystem.beforeStart();
    }

    public void initForAlliance() {
        switch (allianceColor) {
            case RED:
                trajectoryRepo = new RedTrajectoryRepo(drive, playStyle);
                break;
            case BLUE:
                trajectoryRepo = new BlueTrajectoryRepo(drive, playStyle);
                break;
        }

        drive.pose = trajectoryRepo.getStartPose();

        visionSystem.initForAlliance(hardwareMap, allianceColor);
        armSystem.onInit();
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
                        new DelayedAction(0.3, trajectoryRepo.toFirstBackdrop(visionSystem.getPropLocation())),
                        armSystem.raiseArm()
                ),
                clawSystem.openRight());

        action = new SequentialAction(
                clawClose,
                toFloorPlace,
                toFirstBackdrop,
                armSystem.lowerArm()
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
