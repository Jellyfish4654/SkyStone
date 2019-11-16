package org.firstinspires.ftc.teamcode.framework.movement;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;

import java.util.List;
import java.util.Arrays;

/** Implements Moveable with IDriveTrain. */
public class Mecanum implements Moveable {
    private static final double correctionTime = 200;

    private IDriveTrain layer; 
    public Mecanum(DcMotor[] motors, IMU imu, Telemetry telemetry) {
        List<DcMotor> list = Arrays.asList(motors);
        this.layer = new org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum(list, imu, telemetry, list);
    }

    @Override
    public void move(double dist, double dir, double angle, double speed) {
        // TODO: convert dist        
        layer.softEncoderReset();

        double current = layer.getEncoderDistance();
        double target = current + dist;

        double rampUp = current + dist * 0.2;
        double rampDown = current + dist * 0.7;
        double rampEnd = current + dist * 0.8;

        // speed
        double[] pidGain = {0.02, 0.02, 0.02};

        double allowableDistanceError = 0.2;

        while(layer.move(current, dist, rampUp, rampDown, rampEnd, 1.0, 0.1, dir, pidGain, angle, allowableDistanceError, correctionTime)) {
            current = layer.getEncoderDistance();
        }
    }

    @Override
    public void pivot(double angle, double speed) {
        double rampDownAngle = angle * 0.8;
        double allowableError = angle * 0.1;
        while(layer.pivot(angle, rampDownAngle, 1.0, 0.1, correctionTime, allowableError, Direction.FASTEST));
    }

    @Override
    public void stop() {
        layer.stop();
    }
}