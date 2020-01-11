package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.framework.enums.Direction;

public interface DriveTrain {

        public static class DefaultParams {

                public double defaultMaxPower;
                public double defaultMinPower;

                public double[] defaultPIDGain;

                public double defaultCorrectionTime;
                public double defaultAllowableDistanceError;

                public double defaultRampUp;
                public double defaultRampDown;
                public double defaultRampDownEnd;

                public Direction defaultDirection;

                public DefaultParams(double defaultMaxPower, double defaultMinPower, double defaultRampUp,
                                double defaultRampDown, double defaultRampDownEnd, double[] defaultPIDGain,
                                double defaultCorrectionTime, double defaultAllowableDistanceError,
                                Direction defaultDirection) {
                        this.defaultMaxPower = defaultMaxPower;
                        this.defaultMinPower = defaultMinPower;

                        this.defaultRampUp = defaultRampUp;
                        this.defaultRampDown = defaultRampDown;
                        this.defaultRampDownEnd = defaultRampDownEnd;

                        this.defaultPIDGain = defaultPIDGain;

                        this.defaultCorrectionTime = defaultCorrectionTime;
                        this.defaultAllowableDistanceError = defaultAllowableDistanceError;

                        this.defaultDirection = defaultDirection;
                }
        }

        public static class MoveParams {
                /** Angle that robot is facing at end */
                public double endAngle;
                /** Direction that robot should move */
                public double moveAngle;
                /** Distance that robot should move */
                public double targetPosition;

                /** Position where robot stops ramping up */
                public double rampUp;
                /** Position where robot starts ramping down */
                public double rampDown;
                /** Position where robot stops ramping down (at min power) */
                public double rampDownEnd;

                /** Maximum motor power, from 0.0 to 1.0 */
                public double maxPower;
                /** Minimum motor power, from 0.0 to 1.0 */
                public double minPower;

                /** int[3] */
                public double[] pidGain;

                /** Amount of time to spend correcting, in milliseconds. */
                public double correctionTime;

                public double allowableDistanceError;

                public boolean debugMove;

                public MoveParams(double targetPosition, double moveAngle, double endAngle, DefaultParams defaults) {
                        this.targetPosition = targetPosition;
                        this.moveAngle = moveAngle;
                        this.endAngle = endAngle;

                        this.maxPower = defaults.defaultMaxPower;
                        this.minPower = defaults.defaultMinPower;

                        this.rampUp =defaults.defaultRampUp;
                        this.rampDown = defaults.defaultRampDown;
                        this.rampDownEnd = defaults.defaultRampDownEnd;

                        //this.setRamping(defaults.defaultRampUp, defaults.defaultRampDown, defaults.defaultRampDownEnd);

                        this.pidGain = defaults.defaultPIDGain;

                        this.correctionTime = defaults.defaultCorrectionTime;
                        this.allowableDistanceError = defaults.defaultAllowableDistanceError;

                        this.debugMove = false;
                }

                public void setRamping(double rampUp, double rampDown, double rampDownEnd) {
                        this.rampUp = rampUp > targetPosition ? targetPosition : rampUp;

                        this.rampDown = rampDown > targetPosition ? targetPosition : rampDown;

                        this.rampDownEnd = rampDownEnd > targetPosition ? targetPosition : rampDown;
                }
        }

        /**
         * @param currentPosition The current robot position, returned by
         *                        getEncoderDistance()
         * @param params          Parameters
         * @return whether the robot is done moving
         */
        boolean move(double currentPosition, MoveParams params);

        public static class PivotParams {
                public double endAngle;
                public double rampDown;
                public double maxPower;
                public double minPower;
                public double correctionTime;
                public double correctionAngleError;
                public Direction direction;

                public PivotParams(double endAngle, DefaultParams defaults) {
                        this.endAngle = endAngle;
                        this.direction = defaults.defaultDirection;

                        this.rampDown = 90;

                        this.maxPower = defaults.defaultMaxPower;
                        this.minPower = defaults.defaultMinPower;

                        this.correctionTime = defaults.defaultCorrectionTime;
                        this.correctionAngleError = 5;
                }
        }

        /**
         * Pivots the robot to a desired angle. It uses a proportional control loop to
         * maintain the robot's speed
         * 
         * @param params Parameters
         * @return whether the robot is done moving
         */
        boolean pivot(PivotParams params);

        boolean widePivot(PivotParams params);

        /**
         * Reset encoder positions variables to 0.
         */
        void softEncoderReset();

        /**
         * Reset encoder positions variables to 0 and sets motor run modes, making
         * hardware return also 0.
         */
        void resetEncoders();

        /**
         * Gets current encoder distance, and subtracts origin positions
         * 
         * @return List with total encoder distances
         */
        double getEncoderDistance();

        /**
         * Stops all running drivetrain motors
         */
        void stop();

}