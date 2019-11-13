package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;

public abstract class BaseOpMode extends OpMode {
//  protected Servo claw, colorArm, clawPitch, glyph, hugLeft, hugRight;
//  protected ColorSensor color;
//  protected DcMotor hug;
    protected DcMotor mFR, mBR, mFL, mBL;
    protected BNO055IMU imu;

    @Override
    public void init() {
        mFR = hardwareMap.dcMotor.get("fr");
        mBR = hardwareMap.dcMotor.get("br");
        mFL = hardwareMap.dcMotor.get("fl");
        mBL = hardwareMap.dcMotor.get("bl");

        // Allows motor to utilize encoder
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }
}