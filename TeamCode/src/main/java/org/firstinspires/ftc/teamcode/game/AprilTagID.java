package org.firstinspires.ftc.teamcode.game;

public enum AprilTagID {
    BLUE_ALLIANCE_LEFT(1),
    BLUE_ALLIANCE_CENTER(2),
    BLUE_ALLIANCE_RIGHT(3),
    RED_ALLIANCE_LEFT(4),
    RED_ALLIANCE_CENTER(5),
    RED_ALLIANCE_RIGHT(6),

    BLUE_STACK(9), // Aligned with back left pixel stack
    RED_STACK(8),

    BLUE_NAVIGATION(10),
    RED_NAVIGATION(7);

    private final int id;

    AprilTagID(int id) {
        this.id = id;
    }

    public int getValue() {
        return id;
    }

    public static AprilTagID fromInt(int id) {
        for (AprilTagID tag : AprilTagID.values()) {
            if (tag.getValue() == id) {
                return tag;
            }
        }
        return null;
    }
}
