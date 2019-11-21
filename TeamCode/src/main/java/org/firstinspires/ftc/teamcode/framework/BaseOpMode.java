package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.enums.Corner;
import org.firstinspires.ftc.teamcode.framework.enums.StonePosition;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor[] motors;
    protected IMU imu;
    protected DoubleLogger logger;

    protected void initHardware() {
        logger.addData("Status - ", "Intitalizing Hardware");
        motors = new DcMotor[]{
            hardwareMap.dcMotor.get("fr"),
            hardwareMap.dcMotor.get("br"),
            hardwareMap.dcMotor.get("fl"),
            hardwareMap.dcMotor.get("bl")
        };

        motors[Corner.FR].setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize imu
        logger.addData("Status - ", "Initializing IMU");
        BNO055IMU imuHardware = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(imuHardware);

        logger = new DoubleLogger(telemetry);
    }
}