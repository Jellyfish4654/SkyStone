package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseOpMode extends OpMode {
//  protected Servo claw, colorArm, clawPitch, glyph, hugLeft, hugRight;
//  protected ColorSensor color;
//  protected DcMotor hug;
    protected DcMotor mFR, mBR, mFL, mBL;

    public void init() {
        mFR = hardwareMap.dcMotor.get("fr");
        mBR = hardwareMap.dcMotor.get("br");
        mFL = hardwareMap.dcMotor.get("fl");
        mBL = hardwareMap.dcMotor.get("bl");

        // Allows motor to utilize encoder
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFL.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {
    }

    protected void setPowers(double rightFront, double rightBack, double leftFront, double leftBack) {
        this.mFR.setPower(rightFront);
        this.mBR.setPower(rightBack);
        this.mFL.setPower(leftFront);
        this.mBL.setPower(leftBack);
    }

    protected void setMecanumPowers(double angle, double power) {
        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);

        setPowers(power * sin, power * cos, power * cos, power * sin);
    }
}