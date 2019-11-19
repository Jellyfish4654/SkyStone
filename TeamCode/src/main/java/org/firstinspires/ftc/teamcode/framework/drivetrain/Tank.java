package org.firstinspires.ftc.teamcode.framework.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.Utility;

import java.util.List;

/** Implements IDriveTrain without using Odometry. */
public class Tank implements IDriveTrain {
    private List<DcMotor> motors;
    private IMU imu;

    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;
    private ElapsedTime distanceCorrectionTimer;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    private double frEncoder = 0;
    private double brEncoder = 0;
    private double flEncoder = 0;
    private double blEncoder = 0;

    Telemetry telemetry;

    /**
     * Constructor function
     * 
     * @param motors array of 4 motors. see Corners for order
     */
    public Tank(List<DcMotor> motors, IMU imu, Telemetry telemetry) {
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    /*
     * @Override public void move(double dist, double dir, double angle, double
     * speed) throws InterruptedException { // Turn to dir this.pivot(dir, speed);
     * 
     * // TODO: Trial + Error int encoderDist = (int)(dist * 50);
     * 
     * for (DcMotor motor: motors) { motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     * motor.setTargetPosition(motor.getCurrentPosition() + encoderDist);
     * motor.setPower(speed); }
     * 
     * while (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() ||
     * motors[3].isBusy());
     * 
     * this.pivot((angle - dir) % 360, speed); }
     * 
     * @Override public void pivot(double angle, double speed) throws
     * InterruptedException { angle = angle % 360; double currentAngle =
     * imu.getXAngle() % 360; double targetAngle = (currentAngle + angle) % 360;
     * 
     * double dir = angle > 180 ? -1 : 1;
     * 
     * while (true) { currentAngle = imu.getXAngle() % 360;
     * 
     * // TODO: match up this line double diff = dir*(targetAngle - currentAngle) %
     * 360; if (diff > 180) { dir = -dir; continue; }
     * 
     * double power; if (diff > 45) { power = speed; } else { power = speed * (diff
     * / 45); }
     * 
     * // TODO: with this line motors[Corner.FR].setPower(dir*power);
     * motors[Corner.BR].setPower(dir*power);
     * motors[Corner.FL].setPower(-dir*power);
     * motors[Corner.BL].setPower(-dir*power); } }
     */
    @Override
    public boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition,
            double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower, double moveAngle,
            double[] PIDGain, double endOrientationAngle, double allowableDistanceError, double correctiontime) {
        double positionDifference = targetPosition - currentPosition;
        if (Math.abs(positionDifference) <= allowableDistanceError) {
            this.stop();

            if (!targetReached) {
                targetReached = true;
                distanceCorrectionTimer.reset();
                return true;
            }
        } else {
            double rampDownDifference = targetPosition - rampDownTargetPosition;
            double rampDownEndDifference = targetPosition - rampDownEnd;

            double power;
            if (rampDownEndDifference >= Math.abs(positionDifference)) {
                power = lowPower;
                // if current position is between rampDown and rampEnd gradually ramp down
            } else if (rampDownDifference > Math.abs(positionDifference)) {
                power = Math.abs((positionDifference) - rampDownDifference)
                        * ((maxPower - lowPower) / (rampDownDifference - rampDownEndDifference)) + maxPower;
                // if current position is before rampEnd, set to max power
            } else {
                power = maxPower;
            }

            // get current IMU angle **MAY NEED TO ALTER ANGLE BASED ON HUB ORIENTATION**
            double currentAngle = imu.getZAngle(endOrientationAngle);

            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);

            this.setPowerAll(power + pivotCorrection, power + pivotCorrection, power - pivotCorrection,
                    power - pivotCorrection);
        }
        if (targetReached && distanceCorrectionTimer.milliseconds() >= correctionTime) {
            this.stop();
            targetReached = false;
            return false;
        }
        return true;
    }

    @Override
    public boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower,
            double correctionTime, double correctionAngleError, Direction direction) {
        double currentAngle = imu.getZAngle(desiredAngle);
        double angleDifference = desiredAngle - currentAngle;
        double rampDownDifference = desiredAngle - rampDownAngle;
        double power;

        // calculate power
        if (Math.abs(angleDifference) > Math.abs(rampDownDifference)) {
            power = maxPower;
        } else {
            power = (maxPower - minPower) / (Math.abs(rampDownDifference)) * Math.abs(angleDifference) + minPower;
        }
        // turn clockwise or counterclockwise depending on which side of desired angle
        // current angle is
        if (direction == Direction.FASTEST || targetReached) {
            if (angleDifference > 0) {
                this.setPowerAll(-power, -power, power, power);
            } else {
                this.setPowerAll(power, power, -power, -power);
            }
        } else if (direction == Direction.CLOCKWISE) {
            this.setPowerAll(-power, -power, power, power);
        } else {
            this.setPowerAll(power, power, -power, -power);
        }

        // determine if the pivoting angle is in the desired range
        if (Math.abs(angleDifference) < correctionAngleError && !targetReached) {
            pivotTime.reset();
            targetReached = true;
        }
        if (targetReached && pivotTime.milliseconds() >= correctionTime) {
            targetReached = false;
            this.stop();
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void softEncoderReset() {
        frEncoder = motors.get(0).getCurrentPosition();
        brEncoder = motors.get(1).getCurrentPosition();
        flEncoder = motors.get(2).getCurrentPosition();
        blEncoder = motors.get(4).getCurrentPosition();
    }

    @Override
    public void resetEncoders() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    @Override
    public double getEncoderDistance() {
        int encoderSum = 0;
        encoderSum += (motors.get(0).getCurrentPosition() - frEncoder);
        encoderSum += (motors.get(1).getCurrentPosition() - brEncoder);
        encoderSum += (motors.get(2).getCurrentPosition() - flEncoder);
        encoderSum += (motors.get(3).getCurrentPosition() - blEncoder);
        return (encoderSum/motors.size);
    }

    @Override
    public void stop() {
        this.setPowerAll(0, 0, 0, 0);
    }

    private void setPowerAll(double frPower, double brPower, double flPower, double blPower) {
        motors.get(0).setPower(frPower);
        motors.get(1).setPower(brPower);
        motors.get(2).setPower(flPower);
        motors.get(3).setPower(blPower);
    }
}