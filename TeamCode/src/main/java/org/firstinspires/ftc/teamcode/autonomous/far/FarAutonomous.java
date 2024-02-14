package org.firstinspires.ftc.teamcode.autonomous.far;

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
import org.firstinspires.ftc.teamcode.autonomous.ArmTimeoutException;
import org.firstinspires.ftc.teamcode.autonomous.ClawSystem;
import org.firstinspires.ftc.teamcode.autonomous.ParkingLocation;
import org.firstinspires.ftc.teamcode.autonomous.VisionSystem;
import org.firstinspires.ftc.teamcode.autonomous.WristSystem;
import org.firstinspires.ftc.teamcode.base.Mode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.game.AllianceColor;
import org.firstinspires.ftc.teamcode.game.PropLocation;
import org.firstinspires.ftc.teamcode.input.Button;
import org.firstinspires.ftc.teamcode.input.GrizzlyGamepad;

@Config
@Autonomous(name = "Far Autonomous", group = "Autonomous")
public class FarAutonomous extends Mode {

    public static int CAMERA_TIME = 2;
    public static int MAX_DELAY = 10;

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
    private ParkingLocation parkingLocation = ParkingLocation.INNER;
    private int startDelay = 5;

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

            telemetry.addData("Start Delay", startDelay);
            telemetry.addLine("Press DPad up to increase delay, DPad down to decrease delay");
            if (gamepad.getButton(Button.DPAD_UP)) {
                startDelay++;
            } else if (gamepad.getButton(Button.DPAD_DOWN)) {
                startDelay--;
            }
            startDelay = Math.max(CAMERA_TIME, Math.min(startDelay, MAX_DELAY));

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
        } else {
            visionSystem.update();
        }

        drive.updatePoseEstimate();
        armSystem.beforeStart();
    }

    public void initForAlliance() {
        visionSystem.initForAlliance(hardwareMap, allianceColor);
        armSystem.onInit();
    }

    private ElapsedTime timeSinceStart;
    private boolean createdActions = false;

    @Override
    public void onStart() {
        super.onStart();
        timeSinceStart = new ElapsedTime();

        if (!confirmedAlliance) {
            initForAlliance(); // Shouldn't happen, but just in case
            return;
        }
    }

    private void createActions() {
        visionSystem.stopStreaming();

        PropLocation propLocation = visionSystem.getPropLocation();
        switch (allianceColor) {
            case RED:
                trajectoryRepo = new RedTrajectoryRepo(drive, propLocation);
                break;
            case BLUE:
                trajectoryRepo = new BlueTrajectoryRepo(drive, propLocation);
                break;
        }

        drive.pose = trajectoryRepo.getStartPose();

        Action toFloorPlace = new SequentialAction(
                trajectoryRepo.toFloorPlace(),
                clawSystem.openLeft()
        );

        Action toWaitPosition = new SequentialAction(
                new ParallelAction(
                        new DelayedAction(0.3, trajectoryRepo.toWaitPosition()),
                        armSystem.setAngle(0)
                ),
                new DelayedAction(startDelay));

        Action toIntermediatePosition = trajectoryRepo.toIntermediatePosition();

        Action toBackdrop = new SequentialAction(
                new ParallelAction(
                        trajectoryRepo.toBackdrop(),
                        armSystem.raiseArm()
                ),
                clawSystem.openRight());

        action = new SequentialAction(
                toFloorPlace,
                toWaitPosition,
                toIntermediatePosition,
                toBackdrop,
                armSystem.setAngle(0),
                trajectoryRepo.toPark(parkingLocation),
                armSystem.lowerArm()
        );

        createdActions = true;
    }

    private double lastTime = 0;

    @Override
    public void tick(TelemetryPacket packet) {
        super.tick(packet);

        double angle = armSystem.getAngle();
        wristSystem.update(angle);

        double currentTime = timeSinceStart.seconds();

        if (currentTime < CAMERA_TIME) {
            visionSystem.update();
            return;
        }

        if (!createdActions) {
            createActions();
        }

        if (currentTime < startDelay) {
            telemetry.addData("Time Until Start", startDelay - currentTime);
            return;
        }


        drive.updatePoseEstimate();
        try {
            action.run(packet);
        } catch (ArmTimeoutException e) {
            action = armSystem.lowerArm();
        }

        telemetry.addData("Pose", drive.pose);
        telemetry.addData("Prop Location", visionSystem.getPropLocation());

        telemetry.addData("Loop Time", currentTime - lastTime);
        lastTime = currentTime;

        telemetry.addData("Arm Angle", Math.toDegrees(angle));
    }

}
