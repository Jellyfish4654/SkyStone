package org.firstinspires.ftc.teamcode.framework.movement;

/** Provides abstraction over IDriveTrain layer. */
public interface Moveable {
    /** Moves the robot.
     *
     * @param dist Magnitude, or distance, from the current robot position (unit unspecified).
     * @param dir Direction from current robot orientation, in degrees.
     * @param angle How much the robot should turn, in degrees.
     * @param speed How fast the robot shoould move, from 0.0 to 1.0.
     */
    void move(double dist, double dir, double angle, double speed) throws InterruptedException;

    /** Pivots the robot.
     *
     * @param angle How much the robot should turn, in degrees.
     * @param speed How fast the robot shoould turn, from 0.0 to 1.0. 
     */
    void pivot(double angle, double speed) throws InterruptedException;

    /** Stop all motion. */
    void stop();
}