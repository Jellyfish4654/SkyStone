package org.firstinspires.ftc.teamcode.framework.movement;

import com.qualcomm.robotcore.hardware.DcMotor;

/** Implements Moveable without using Odometry. */
public class Tank implements Moveable {
    DcMotor mFR, mBR, mFL, mBL;
    public Tank(DcMotor mFR, DcMotor mBR, DcMotor mFL, DcMotor mBL) {
        this.mFR = mFR;
        this.mBR = mBR;
        this.mFL = mFL;
        this.mBL = mBL;
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