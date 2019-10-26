package org.firstinspires.ftc.teamcode.framework.movement;

/** Provides abstraction over IDriveTrain layer. */
public interface Moveable {
    /** Moves the robot.
     *
     * @param dist Magnitude, or distance, from the current robot position (unit unspecified).
     * @param dir Direction from current robot orientation.
     * @param angle How much the robot should turn, in degrees.
     */
    void move(double dist, double dir, double angle);

    /** Pivots the robot.
     *
     * @param angle How much the robot should turn, in degrees.
     */
    void pivot(double angle);

    /** Stop all motion. */
    void stop();
}