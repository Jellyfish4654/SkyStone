package org.firstinspires.ftc.teamcode.framework.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

/** Represents the foundation grabber subsystem. */
public class Foundation {
    private final static double LEFT_RETRACT = 0;
    private final static double LEFT_EXTEND = 1;
    private final static double RIGHT_RETRACT = 1;
    private final static double RIGHT_EXTEND = 0;

    private Servo left;
    private Servo right;

    public Foundation(Servo left, Servo right) {
        this.left = left;
        this.right = right;
    }

    public void retract() {
        left.setPosition(LEFT_RETRACT);
        right.setPosition(RIGHT_RETRACT);
    }

    public void extend() {
        left.setPosition(LEFT_EXTEND);
        right.setPosition(RIGHT_EXTEND);
    }

    public boolean extended() {
        return left.getPosition() != LEFT_RETRACT;
    }
}