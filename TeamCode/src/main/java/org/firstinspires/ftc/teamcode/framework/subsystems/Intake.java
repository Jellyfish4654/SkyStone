package org.firstinspires.ftc.teamcode.framework.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/** Represents the intake subsystem. */
public class Intake {
    private DcMotor[] motors;
    public Intake(DcMotor[] motors) {
        this.motors = motors;
    }
    
    public void run(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void stop() {
        run(0);
    }
}
