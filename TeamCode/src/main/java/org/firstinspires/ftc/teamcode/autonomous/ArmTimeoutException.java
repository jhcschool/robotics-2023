package org.firstinspires.ftc.teamcode.autonomous;

public class ArmTimeoutException extends RuntimeException {
    public ArmTimeoutException() {
        super("Arm action timed out");
    }
}
