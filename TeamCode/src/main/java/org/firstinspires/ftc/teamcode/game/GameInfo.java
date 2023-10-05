package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.EnumMap;

public class GameInfo {

    private static GameInfo instance = null;
    private EnumMap<AprilTagID, Vector2d> aprilTagPositions = new EnumMap<>(AprilTagID.class);

    private GameInfo() {
//        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_LEFT, new Vector2d(-42, 60));
//        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_CENTER, new Vector2d(-36, 60));
//        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_RIGHT, new Vector2d(-30, 60));
//
//        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_LEFT, new Vector2d(30, 60));
//        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_CENTER, new Vector2d(36, 60));
//        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_RIGHT, new Vector2d(42, 60));
//
//        aprilTagPositions.put(AprilTagID.BACK_LEFT_STACK, new Vector2d(-36, -72));
//        aprilTagPositions.put(AprilTagID.BACK_RIGHT_STACK, new Vector2d(36, -72));
//
//        aprilTagPositions.put(AprilTagID.BACK_LEFT_NAVIGATION, new Vector2d(-36, -72));

        // swap x and y for all
        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_LEFT, new Vector2d(60, 42));
        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_CENTER, new Vector2d(60, 36));
        aprilTagPositions.put(AprilTagID.BLUE_ALLIANCE_RIGHT, new Vector2d(60, 30));

        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_LEFT, new Vector2d(60, -30));
        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_CENTER, new Vector2d(60, -36));
        aprilTagPositions.put(AprilTagID.RED_ALLIANCE_RIGHT, new Vector2d(60, -42));

        aprilTagPositions.put(AprilTagID.BLUE_STACK, new Vector2d(-72, 36));
        aprilTagPositions.put(AprilTagID.RED_STACK, new Vector2d(-72, -36));

        aprilTagPositions.put(AprilTagID.BLUE_NAVIGATION, new Vector2d(-72, 42));
        aprilTagPositions.put(AprilTagID.RED_NAVIGATION, new Vector2d(-72, -42));
    }

    public static GameInfo getInstance() {
        if (instance == null) {
            instance = new GameInfo();
        }
        return instance;
    }

    public Vector2d getAprilTagPosition(AprilTagID id) {
        return aprilTagPositions.get(id);
    }

}
