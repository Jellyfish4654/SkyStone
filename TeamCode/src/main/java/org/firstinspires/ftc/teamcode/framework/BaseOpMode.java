package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor[] motors;
    protected DcMotor[] intake;
    protected IMU imu;
    protected DoubleLogger logger = new DoubleLogger(telemetry);

    public ElapsedTime timer = new ElapsedTime();
    public TFStoneDetector stoneDetector;

    public DebugMode debugMode = DebugMode.NONE;

    protected void initHardware() {
        logger.addData("Status - ", "Intitalizing Hardware");
        motors = new DcMotor[] { hardwareMap.dcMotor.get("fr"), hardwareMap.dcMotor.get("br"),
                hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("bl") };

        intake = new DcMotor[] { hardwareMap.dcMotor.get("il"), hardwareMap.dcMotor.get("ir") };

        for (DcMotor motor : intake) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        motors[Motor.BR].setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize imu
        logger.addData("Status - ", "Initializing IMU");
        BNO055IMU imuHardware = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(imuHardware);
    }

    protected void intake(float power) {
        for (DcMotor motor : intake) {
            motor.setPower(power);
        }
    }
}