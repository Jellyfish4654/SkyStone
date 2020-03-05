package org.firstinspires.ftc.teamcode.framework.drivetrain;

import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.enums.Motors;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Mecanum implements DriveTrain {
	private DcMotor[] motors;
	private DoubleLogger logger;
	private IMU imu;

	public Mecanum(DcMotor[] motors, IMU imu, DoubleLogger logger) {
		this.motors = motors;
		this.imu = imu;
		this.logger = logger;
	}

	private long startTime = -1;

	@Override
	public boolean move(double _c, DriveTrain.MoveParams moveParams) {
		long now = System.currentTimeMillis();
		if (startTime == -1) {
			startTime = now;
		}

		// TODO: Change this constant
		// TODO: incorporate rampup/down
		long requiredTime = (long)(moveParams.targetPosition * 1000);

		if (now - startTime > requiredTime) {
			return true;
		}

		double powerFR = Math.sin(moveParams.moveAngle + Math.PI / 4);
		double powerFL = Math.sin(moveParams.moveAngle + Math.PI / 4);

		double factor = 1;
		if (powerFR > 1) {
			factor *= powerFR;
		}
		if (powerFL / factor > 1) {
			factor *= powerFL;
		}

		powerFR /= factor;
		powerFL /= factor;

		motors[Motors.FR].setPower(powerFR);
		motors[Motors.BL].setPower(powerFR);
		motors[Motors.FL].setPower(powerFL);
		motors[Motors.BR].setPower(powerFL);
		return false;
	}

	// TODO
	@Override public boolean pivot(DriveTrain.PivotParams params) {
		return true;
	}

	@Override public boolean widePivot(DriveTrain.PivotParams params) {
		return pivot(params);
	}

	// We don't use encoders here & the currentPosition arg is ignored anyways, so
    @Override
    public void resetEncoders() {}

	@Override
	public void softEncoderReset() {}

	@Override public double getEncoderDistance() {
		return (double)(System.currentTimeMillis() - startTime);
	}

	@Override public void stop() {
		for (DcMotor motor: motors) {
			motor.setPower(0);
		}
	}
}
