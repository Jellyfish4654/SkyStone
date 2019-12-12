package org.firstinspires.ftc.teamcode.framework.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.Utility;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

import java.util.List;

/** Implements DriveTrain without using Odometry. */
public class Tank implements DriveTrain {
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

    DoubleLogger logger;

    /**
     * Constructor function
     * 
     * @param motors List of 4 motors.
     */
    public Tank(List<DcMotor> motors, IMU imu, DoubleLogger logger) {
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.logger = logger;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();
    }

    @Override
    public boolean move(double currentPosition, DriveTrain.MoveParams params) {
        double positionDifference = params.targetPosition - currentPosition;
        if (Math.abs(positionDifference) <= params.allowableDistanceError) {
            this.stop();

            if (!targetReached) {
                targetReached = true;
                distanceCorrectionTimer.reset();
                return true;
            }
        } else {
            double rampDownDifference = params.targetPosition - params.rampDown;
            double rampDownEndDifference = params.targetPosition - params.rampDownEnd;

            double power;
            if (rampDownEndDifference >= Math.abs(positionDifference)) {
                power = params.minPower;
                // if current position is between rampDown and rampEnd gradually ramp down
            } else if (rampDownDifference > Math.abs(positionDifference)) {
                power = Math.abs((positionDifference) - rampDownDifference)
                        * ((params.maxPower - params.minPower) / (rampDownDifference - rampDownEndDifference))
                        + params.maxPower;
                // if current position is before rampEnd, set to max power
            } else {
                power = params.maxPower;
            }

            // get current IMU angle **MAY NEED TO ALTER ANGLE BASED ON HUB ORIENTATION**
            double currentAngle = imu.getZAngle(params.endAngle);

            double pivotCorrection = ((currentAngle - params.endAngle) * params.pidGain[0]);

            this.setPowerAll(power + pivotCorrection, power + pivotCorrection, power - pivotCorrection,
                    power - pivotCorrection);
        }
        if (targetReached && distanceCorrectionTimer.milliseconds() >= params.correctionTime) {
            this.stop();
            targetReached = false;
            return false;
        }
        return true;
    }

    @Override
    public boolean pivot(DriveTrain.PivotParams params) {
        double currentAngle = imu.getZAngle(params.endAngle);
        double angleDifference = params.endAngle - currentAngle;
        double rampDownDifference = params.endAngle - params.rampDown;
        double power;

        // calculate power
        if (Math.abs(angleDifference) > Math.abs(rampDownDifference)) {
            power = params.maxPower;
        } else {
            power = (params.maxPower - params.minPower) / (Math.abs(rampDownDifference)) * Math.abs(angleDifference)
                    + params.minPower;
        }
        // turn clockwise or counterclockwise depending on which side of desired angle
        // current angle is
        if (params.direction == Direction.FASTEST || targetReached) {
            if (angleDifference > 0) {
                this.setPowerAll(-power, -power, power, power);
            } else {
                this.setPowerAll(power, power, -power, -power);
            }
        } else if (params.direction == Direction.CLOCKWISE) {
            this.setPowerAll(-power, -power, power, power);
        } else {
            this.setPowerAll(power, power, -power, -power);
        }

        // determine if the pivoting angle is in the desired range
        if (Math.abs(angleDifference) < params.correctionAngleError && !targetReached) {
            pivotTime.reset();
            targetReached = true;
        }
        if (targetReached && pivotTime.milliseconds() >= params.correctionTime) {
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
        return (encoderSum / motors.size());
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