package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseOpMode extends OpMode {
    protected Servo claw, colorArm, clawPitch, glyph, hugLeft, hugRight;
    protected ColorSensor color;
    protected DcMotor backLeftDrive, backRightDrive, frontLeftDrive, frontRightDrive, hug;

    public void init() {
        frontLeftDrive = hardwareMap.dcMotor.get("lf");
        frontRightDrive = hardwareMap.dcMotor.get("rf");
        backLeftDrive = hardwareMap.dcMotor.get("lb");
        backRightDrive = hardwareMap.dcMotor.get("rb");

        // Allows motor to utilize encoder
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {
    }

    protected void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        this.frontLeftDrive.setPower(leftFront);
        this.backLeftDrive.setPower(leftBack);
        this.frontRightDrive.setPower(rightFront);
        this.backRightDrive.setPower(rightBack);
    }

    protected void setMecanumPowers(double angle, double power) {
        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);

        setPowers(power * sin, power * cos, power * cos, power * sin);
    }
}