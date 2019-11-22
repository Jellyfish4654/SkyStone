package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.framework.enums.Direction;

public interface DriveTrain {
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

                public MoveParams(double targetPosition, double moveAngle, double endAngle) {
                        this.targetPosition = targetPosition;
                        this.moveAngle = moveAngle;
                        this.endAngle = endAngle;

                        this.rampUp = 0.15 * targetPosition;
                        this.rampDown = 0.85 * targetPosition;
                        this.rampDownEnd = 0.95 * targetPosition;

                        this.maxPower = 1.0;
                        this.minPower = 0.1;

                        this.pidGain = new double[]{0.01, 0.01, 0.01};

                        this.correctionTime = 500;
                        this.allowableDistanceError = 1;
                }
        }

        /**
         * @param currentPosition The current robot position, returned by getEncoderDistance()
         * @param params Parameters
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

                public PivotParams(double endAngle, Direction direction) {
                        this.endAngle = endAngle;
                        this.direction = direction;

                        this.rampDown = (endAngle - 30) % 360;

                        this.maxPower = 1.0;
                        this.minPower = 0.1;

                        this.correctionTime = 500;
                        this.correctionAngleError = 10;
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