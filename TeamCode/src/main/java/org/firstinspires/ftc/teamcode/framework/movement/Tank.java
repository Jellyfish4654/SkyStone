package org.firstinspires.ftc.teamcode.framework.movement;

import com.qualcomm.robotcore.hardware.DcMotor;

/** Implements Moveable without using Odometry. */
public class Tank implements Moveable {
    DcMotor mLF, mLB, mRF, mRB;
    public Tank(DcMotor mLF, DcMotor mLB, DcMotor mRF, DcMotor mRB) {
        this.mLF = mLF;
        this.mLB = mLB;
        this.mRF = mRF;
        this.mRB = mRB;
    }

    @Override
    public void move(double dist, double dir, double angle) {
    }

    @Override
    public void pivot(double angle) {
    }

    @Override
    public void stop() {

    }
}