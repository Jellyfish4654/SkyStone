package org.firstinspires.ftc.teamcode.framework.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/** Represents the intake subsystem. */
public class Intake {
    private final static double INTAKE_POWER = 0.8;
    private final static double OUTAKE_POWER = -0.8;

    private final static double FRONT_OPEN = 1;
    private final static double FRONT_BLOCK = 0;

    private final static double BACK_OPEN = 1;
    private final static double BACK_BLOCK = 0;

    private final static double WACKER_RETRACT = 1;
    private final static double WACKER_EXTEND = 0;

    private DcMotor[] motors;
    Servo frontBlock;
    Servo wacker;
    Servo backBlock;
    public Intake(DcMotor[] motors, Servo frontBlock, Servo wacker, Servo backBlock) {
        this.motors = motors;
        this.frontBlock = frontBlock;
        this.wacker = wacker;
        this.backBlock = backBlock;

        frontBlock.setPosition(FRONT_BLOCK);
        backBlock.setPosition(BACK_BLOCK);
        wacker.setPosition(WACKER_RETRACT);
    }

    private static enum StoneState {
        NONE,
        INTAKE,
        PLATFORM
    }


    /** NONE if stone is not in robot,
        INTAKE in stone is either currently being intaked or sitting right above intake
        PLATFORM if stone is on the platform
    */
    private StoneState state;

    private long intakeStartTime;
    private long outputStartTime;

    /* A call to output() will restart the intake() process. */
    /* A call to intake() will restart the output() process. */
    /* A call to intake(true) must not be followed by intake(false) unless the process was completed, and vice versa */
    public boolean intake(boolean block) {
        if(true) return true;

        outputStartTime = -1;

        long now = System.currentTimeMillis();
        // t=0: set front servo position, set stonestate to intake
        if (intakeStartTime == -1) {
            intakeStartTime = now;
            state = StoneState.INTAKE;

            // Reset servo positions, incase this was triggered halfway through output
            backBlock.setPosition(BACK_BLOCK);
            wacker.setPosition(WACKER_RETRACT);
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }

            if (block) {
                frontBlock.setPosition(FRONT_BLOCK);
            } else {
                frontBlock.setPosition(FRONT_OPEN);
            }
        }

        // t=100: turn on intake motors
        if (now - intakeStartTime >= 200 && motors[0].getPower() != INTAKE_POWER) {
            for (DcMotor motor : motors) {
                motor.setPower(INTAKE_POWER);
            }
        }

        if (block) {
            // t=1000: assume stone is above intake, stop motors, done
            if (now - intakeStartTime >= 1000 && motors[0].getPower() != 0) {
                for (DcMotor motor : motors) {
                    motor.setPower(0);
                }

                intakeStartTime = -1;
                return true;
            }
        } else {
            // t=1500: assume stone is on platform, turn off motors, set stonestate to platform
            if (now - intakeStartTime >= 1500 && motors[0].getPower() != 0) {
                for (DcMotor motor : motors) {
                    motor.setPower(0);
                }
                state = StoneState.PLATFORM;
            }

            // t=1600: lock front, done
            if (now - intakeStartTime >= 1600 && frontBlock.getPosition() == FRONT_OPEN) {
                frontBlock.setPosition(FRONT_BLOCK);

                intakeStartTime = -1;
                return true;
            }
        }

        return false;
    }

    public boolean output() {
        if(true) return true;

        intakeStartTime = -1;

        long now = System.currentTimeMillis();
        if (outputStartTime == -1) {
            outputStartTime = now;

            // reset servo positions/motor speed 
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
            frontBlock.setPosition(FRONT_BLOCK);
            wacker.setPosition(WACKER_RETRACT);

            if (state == StoneState.PLATFORM) {
                backBlock.setPosition(BACK_OPEN);
            } else if (state == StoneState.INTAKE) {
                backBlock.setPosition(BACK_BLOCK);

                // INTAKE t=0: turn motors on backwards 
                for (DcMotor motor : motors) {
                    motor.setPower(OUTAKE_POWER);
                }
            } else {
                backBlock.setPosition(BACK_BLOCK);

                outputStartTime = -1;
                return true;
            }
        }

        if (state == StoneState.PLATFORM) {
            // PLATFORM t=500: wack the block
            if (now - outputStartTime >= 500 && motors[0].getPower() != 0) {
                wacker.setPosition(WACKER_EXTEND);
            }
            // PLATFORM t=1500: reset, done
            if (now - outputStartTime >= 1500 && motors[0].getPower() != 0) {
                // reset servos
                wacker.setPosition(WACKER_RETRACT);
                backBlock.setPosition(BACK_BLOCK);

                state = StoneState.NONE;
                outputStartTime = -1;
                return true;
            }
        } else if (state == StoneState.INTAKE) {
            // INTAKE t=1000: stop power, done
            if (now - outputStartTime >= 1000 && motors[0].getPower() != 0) {
                for (DcMotor motor : motors) {
                    motor.setPower(0);
                }

                state = StoneState.NONE;
                outputStartTime = -1;
                return true;
            }
        }

        return false;
    }
}
