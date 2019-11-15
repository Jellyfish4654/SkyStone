package org.firstinspires.ftc.teamcode.framework.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.enums.Corner;

/** Implements Moveable without using Odometry. */
public class Tank implements Moveable {
    DcMotor[] motors;
    DcMotor[] encoders;
    IMU imu;
    public Tank(DcMotor[] motors, IMU imu) {
        this.motors = motors;
        this.encoders = motors;
    }

    @Override
    public void move(double dist, double dir, double angle, double speed) throws InterruptedException {
        // Turn to dir
        this.pivot(dir, speed);

        // TODO: which direction
        motors[Corner.FR].setPower(speed);
        motors[Corner.FL].setPower(speed);
        motors[Corner.BR].setPower(-speed);
        motors[Corner.BL].setPower(-speed);

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }

        this.pivot((angle - dir) % 360, speed);
    }

    @Override
    public void pivot(double angle, double speed) throws InterruptedException {
        // TODO: which direction
        motors[Corner.FR].setPower(speed);
        motors[Corner.BR].setPower(-speed);

        motors[Corner.FL].setPower(-speed);
        motors[Corner.BL].setPower(speed);

        // TODO: find correct formula
        Thread.sleep((long)(5 * angle / speed));

        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }

    @Override
    public void stop() {
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }
}