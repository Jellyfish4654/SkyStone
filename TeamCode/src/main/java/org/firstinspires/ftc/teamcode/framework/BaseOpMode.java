package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.enums.Corner;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor[] motors;
    protected IMU imu;

    protected void initHardware() {
        motors = new DcMotor[]{
            hardwareMap.dcMotor.get("fr"),
            hardwareMap.dcMotor.get("br"),
            hardwareMap.dcMotor.get("fl"),
            hardwareMap.dcMotor.get("bl")
        };

        motors[Corner.FR].setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize imu
        BNO055IMU imuHardware = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(imuHardware);
    }
}