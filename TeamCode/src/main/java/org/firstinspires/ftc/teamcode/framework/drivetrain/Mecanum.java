package org.firstinspires.ftc.teamcode.framework.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.Utility;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

import java.util.List;

public class Mecanum implements DriveTrain {
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IMU imu;

    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;
    private ElapsedTime distanceCorrectionTimer;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    private double leftVerticalLastEncoder = 0;
    private double rightVerticalLastEncoder = 0;
    private double horizontalLastEncoder = 0;

    private int leftVerticalMultiplier = 1;
    private int rightVerticalMultiplier = 1;
    private int normalMultiplier = 1;

    DoubleLogger logger;

    /**
     * Constructor for mecanum drivetrain with free-spinning odometry wheels
     * 
     * @param motors   List of motors on drivetrain in order of Right Front, Right
     *                 Back, Left Front and then Left Back
     * @param imu      the inertial measurement unit or gyro sensor of the robot
     * @param encoders List of encoders (passed in as DcMotor) on drivetrain to
     *                 calculate distances and positions
     */
    public Mecanum(List<DcMotor> motors, IMU imu, DoubleLogger logger, List<DcMotor> encoders) {
        this.motors = motors;
        this.imu = imu;
        // this.imu.initialize();
        this.logger = logger;
        this.encoders = encoders;
        pivotTime = new ElapsedTime();
        distanceCorrectionTimer = new ElapsedTime();

        reverseLeftEncoder();
        reverseRightEncoder();
    }

    /**
     * @param frPower right front power
     * @param brPower right back power
     * @param flPower left front power
     * @param blPower left back power
     */
    private void setPowerAll(double frPower, double brPower, double flPower, double blPower) {
        motors.get(0).setPower(frPower);
        motors.get(1).setPower(brPower);
        motors.get(2).setPower(flPower);
        motors.get(3).setPower(blPower);
    }

    // translation of vertical, horizontal, pivot power into motor speeds
    public void rawSlide(double horizontal, double vertical, double pivot, double maxPower) {
        double powers[] = { vertical - horizontal + pivot, vertical + horizontal + pivot, vertical + horizontal - pivot,
                vertical - horizontal - pivot };

        // Only do speed changes if there is positional movement
        if (horizontal != 0 || vertical != 0) {
            int max = 0;
            int counter = 0;

            for (double element : powers) {
                if (Math.abs(element) > Math.abs(powers[max])) {
                    max = counter;
                }
                counter++;
            }

            double maxCalculatedPower = Math.abs(powers[max]);

            if (maxCalculatedPower != 0) {
                powers[0] = powers[0] / maxCalculatedPower * maxPower;
                powers[1] = powers[1] / maxCalculatedPower * maxPower;
                powers[2] = powers[2] / maxCalculatedPower * maxPower;
                powers[3] = powers[3] / maxCalculatedPower * maxPower;

            }
        }
        this.setPowerAll(powers[0], powers[1], powers[2], powers[3]);
    }

    // calculate power for x direction
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    // calculate power for y direction
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    @Override
    public void resetEncoders() {
        for (DcMotor motor : this.encoders) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }

    public void resetEncoders(DcMotor.RunMode endMode) {
        for (DcMotor motor : this.encoders) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(endMode);
        }
        leftVerticalLastEncoder = 0;
        rightVerticalLastEncoder = 0;
        horizontalLastEncoder = 0;
    }

    // checks whether it needs to continue moving or stop, then calls rawSlide
    public boolean move(double currentPosition, DriveTrain.MoveParams params) {
        double positionDifference = params.targetPosition - currentPosition;

        if (params.debugMove) {
            logger.addData("TargetReached", targetReached);
            logger.addData("PositionDiff", positionDifference);
            logger.addData("CurrentPosition", currentPosition);
            logger.addData("Moveangle", params.moveAngle);
            logger.addData("IMU data", imu.getZAngle(params.endAngle));
            logger.addData("Imu x", imu.getXAngle());
            logger.addDataUpdate("Imu y", imu.getYAngle());
        }

        // if it's within allowableDistanceError of the end, stop moving
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
            if (rampDownEndDifference <= Math.abs(positionDifference)) {
                power = params.minPower;
                logger.addData("power", "min");
                // if current position is between rampDown and rampEnd gradually ramp down
            } else if (rampDownDifference < Math.abs(positionDifference)) {
                power = Math.abs((positionDifference) - rampDownDifference)
                        * ((params.maxPower - params.minPower) / (rampDownDifference - rampDownEndDifference))
                        + params.maxPower;
                logger.addData("power ramp down", power);
                // if current position is before rampEnd, set to max power
            } else {
                power = params.maxPower;
                logger.addData("power", "max");
            }

            // get current IMU angle **MAY NEED TO ALTER ANGLE BASED ON HUB ORIENTATION**
            double currentAngle = imu.getZAngle(params.endAngle);

            double moveAngle = params.moveAngle - currentAngle;

            if (moveAngle <= -180) {
                moveAngle += 360;
            }
            if (positionDifference < 0) {
                moveAngle += 180;
            }

            // x vector movement
            double horizontal = Utility.roundTwoDec(calculateX(moveAngle, power));
            // y vector movement
            double vertical = Utility.roundTwoDec(calculateY(moveAngle, power));
            // correction
            double pivotCorrection = ((currentAngle - params.endAngle) * params.pidGain[0]);

            rawSlide(horizontal, vertical, pivotCorrection, power);
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
            logger.addDataUpdate("Pivot Ramp Down", power);
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
    public boolean widePivot(DriveTrain.PivotParams params) {
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
            logger.addDataUpdate("Pivot Ramp Down", power);
        }
        // turn clockwise or counterclockwise depending on which side of desired angle
        // current angle is
        if (params.direction == Direction.FASTEST || targetReached) {
            if (angleDifference > 0) {
                this.setPowerAll(-power, -power, 0, 0);
            } else {
                this.setPowerAll(0, 0, -power, -power);
            }
        } else if (params.direction == Direction.CLOCKWISE) {
            this.setPowerAll(-power, -power, 0, 0);
        } else {
            this.setPowerAll(0, 0, -power, -power);
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
        leftVerticalLastEncoder = (leftVerticalMultiplier * this.encoders.get(0).getCurrentPosition());
        rightVerticalLastEncoder = (rightVerticalMultiplier* this.encoders.get(1).getCurrentPosition());
        horizontalLastEncoder = (normalMultiplier * this.encoders.get(2).getCurrentPosition());
        logger.addData("encoders left", leftVerticalLastEncoder);
        logger.addData("encoders right", rightVerticalLastEncoder);
        logger.addDataUpdate("encoders horizontal", horizontalLastEncoder);
    }

    private double[] getEncoderPositions() {
        double[] encoders = {
                (leftVerticalMultiplier * this.encoders.get(0).getCurrentPosition()) - leftVerticalLastEncoder,
                (rightVerticalMultiplier * this.encoders.get(1).getCurrentPosition()) - rightVerticalLastEncoder,
                (normalMultiplier * this.encoders.get(2).getCurrentPosition()) - horizontalLastEncoder };
        return encoders;
    }

    public double getEncoderDistance() {

        // Get Current Positions
        double[] encoders = this.getEncoderPositions();

        double vlPos = encoders[0];
        double vrPos = encoders[1];
        double hPos = encoders[2];

        // Average the Vertical Wheels
        double y = ((Math.abs(vlPos) + Math.abs(vrPos)) / 2);
        double x = hPos;

        // Calculate distance
        double distance = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        return distance;
    }

    private void reverseLeftEncoder() {
        if (leftVerticalMultiplier == 1) {
            leftVerticalMultiplier = -1;
        } else {
            leftVerticalMultiplier = 1;
        }
    }

    private void reverseRightEncoder() {
        if (rightVerticalMultiplier == 1) {
            rightVerticalMultiplier = -1;
        } else {
            rightVerticalMultiplier = 1;
        }
    }

    private void reverseNormalEncoder() {
        if (normalMultiplier == 1) {
            normalMultiplier = -1;
        } else {
            normalMultiplier = 1;
        }
    }

    @Override
    public void stop() {
        setPowerAll(0, 0, 0, 0);
    }
}
