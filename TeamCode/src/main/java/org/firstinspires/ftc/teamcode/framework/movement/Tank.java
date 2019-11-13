package org.firstinspires.ftc.teamcode.framework.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IIMU;

/** Implements Moveable without using Odometry. */
public class Tank implements Moveable {
    DcMotor mFR, mBR, mFL, mBL;
    IIMU imu;
    public Tank(DcMotor mFR, DcMotor mBR, DcMotor mFL, DcMotor mBL, IIMU imu) {
        this.mFR = mFR;
        this.mBR = mBR;
        this.mFL = mFL;
        this.mBL = mBL;
    }

    @Override
    public void move(double dist, double dir, double angle, double speed) throws InterruptedException {
        // Turn to dir
        this.pivot(dir, speed);

        // TODO: which direction
        mFR.setPower(speed);
        mBR.setPower(-speed);
        mFL.setPower(speed);
        mBL.setPower(-speed);

        // TODO: adjust. prob not going to get 100% accurate
        Thread.sleep((long)(50 * dist / speed));

        mFR.setPower(0);
        mBR.setPower(0);
        mFL.setPower(0);
        mBL.setPower(0);

        this.pivot((angle - dir) % 360, speed);
    }

    @Override
    public void pivot(double angle, double speed) throws InterruptedException {
        // TODO: which direction
        mFR.setPower(speed);
        mBR.setPower(-speed);
        mFL.setPower(-speed);
        mBL.setPower(speed);

        // TODO: find correct formula
        Thread.sleep((long)(5 * angle / speed));

        mFR.setPower(0);
        mBR.setPower(0);
        mFL.setPower(0);
        mBL.setPower(0);
    }

    @Override
    public void stop() {
        mFR.setPower(0);
        mBR.setPower(0);
        mFL.setPower(0);
        mBL.setPower(0);
    }
}