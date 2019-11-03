package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.enums.Direction;

public interface IDriveTrain {
        /**
         * Moves the robot in a direction while maintaining a certain orientation
         *
         * The first five parameters are in the same unit.
         * 
         * @param currentPosition        The current position of robot in any unit eg.
         *                               encoder counts, sensor distances...
         * @param targetPosition         The target position of the robot
         * @param rampUpTargetPosition   Position at which the robot will stop ramping
         *                               up in power
         * @param rampDownTargetPosition Position at which the robot will start ramping
         *                               down
         * @param rampDownEnd            Position at which the robot will stop ramping
         *                               down
         * @param maxPower               The maximum power the robot will move at,
         *                               ranging from 0.0 to 1.0
         * @param lowPower               The lowest power the robot will move at,
         *                               ranging from 0.0 to 1.0
         * @param moveAngle              The angle at which the robot will move in the
         *                               frame of reference of the starting position
         * @param PIDGain                Three gains to control PID feedback loop for
         *                               Orientation correction
         * @param endOrientationAngle    The Direction the robot is facing at the end
         * @param correctionTime         The amount of time to spend correcting to stay
         *                               within the desired range, in milliseconds.
         * @return true if the motion is complete, false is the motion is ongoing
         */
        boolean move(double currentPosition, double targetPosition, double rampDownTargetPosition,
                        double rampUpTargetPosition, double rampDownEnd, double maxPower, double lowPower,
                        double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError,
                        double correctiontime);

        /**
         * Pivots the robot to a desired angle. It uses a proportional control loop to
         * maintain the robot's speed
         * 
         * @param desiredAngle         The angle to which to pivot to
         * @param rampDownAngle        The angle at which to start slowing down
         * @param maxPower             The max power to pivot at
         * @param minPower             The min power to pivot at
         * @param correctionTime       The amount of time to spend correcting to stay
         *                             within the desired range, in milliseconds.
         * @param correctionAngleError
         * @return true if the motion is complete, false is the motion is ongoing
         */
        boolean pivot(double desiredAngle, double rampDownAngle, double maxPower, double minPower,
                        double correctionTime, double correctionAngleError, Direction direction);

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