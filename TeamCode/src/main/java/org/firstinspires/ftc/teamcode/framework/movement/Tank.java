package org.firstinspires.ftc.teamcode.framework.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.enums.Corner;

/** Implements Moveable without using Odometry. */
public class Tank implements Moveable {
    DcMotor[] motors;
    IMU imu;

    /** Constructor function
     * 
     * @param motors array of 4 motors. see Corners for order
     */
    public Tank(DcMotor[] motors, IMU imu) {
        this.motors = motors;
        this.imu = imu;
    }

    @Override
    public void move(double dist, double dir, double angle, double speed) throws InterruptedException {
        // Turn to dir
        this.pivot(dir, speed);

        // TODO: Trial + Error
        int encoderDist = (int)(dist * 50);

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(motor.getCurrentPosition() + encoderDist);
            motor.setPower(speed);
        }

        while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy());

        this.pivot((angle - dir) % 360, speed);
    }

    @Override
    public void pivot(double angle, double speed) throws InterruptedException {
        angle = angle % 360;
        double currentAngle = imu.getXAngle() % 360;
        double targetAngle = (currentAngle + angle) % 360;

        double dir = angle > 180 ? -1 : 1;

        while (true) {
            currentAngle = imu.getXAngle() % 360;

            // TODO: match up this line
            double diff = dir*(targetAngle - currentAngle) % 360;
            if (diff > 180) {
                dir = -dir;
                continue;
            }

            double power;
            if (diff > 45) {
                power = speed;
            } else { 
                power = speed * (diff / 45);
            }

            // TODO: with this line
            motors[Corner.FR].setPower(dir*power);
            motors[Corner.BR].setPower(dir*power);
            motors[Corner.FL].setPower(-dir*power);
            motors[Corner.BL].setPower(-dir*power);
        }
    }

    @Override
    public void stop() {
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }
}