package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.subsystems.GlobalPosition;
import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor front_right, back_right, front_left, back_left;
    protected DcMotor verticalLeft, verticalRight, horizontal, horizontal2;

    protected DcMotor[] motors, encoders;
    protected DcMotor[] intake;

    protected IMU imu;
    protected GlobalPosition globalPositionUpdate;
    protected DoubleLogger logger = new DoubleLogger(telemetry);

    public ElapsedTime timer = new ElapsedTime();

    protected final double countsPerMM = 60 * Math.PI * 8192; // 8192
    protected final double countsPerInch = countsPerMM * 25.4;

    protected void initHardware() {
        logger.addDataUpdate("Status", "Intitalizing Hardware");

        front_right = hardwareMap.dcMotor.get("fr"); // 0
        back_right = hardwareMap.dcMotor.get("br"); // 1
        front_left = hardwareMap.dcMotor.get("fl"); // 2
        back_left = hardwareMap.dcMotor.get("bl"); // 3

        verticalLeft = hardwareMap.dcMotor.get("fr"); // 0
        verticalRight = hardwareMap.dcMotor.get("br"); // 1
        horizontal = hardwareMap.dcMotor.get("fl");// 2
        horizontal2 = hardwareMap.dcMotor.get("bl");// 3 EMPTY

        // back_right.setDirection(DcMotorSimple.Direction.REVERSE); //backright was
        // reversed but honestly it could be that all the others need to be reversed for
        // auto code to work.

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoders = new DcMotor[] { verticalLeft, verticalRight, horizontal, horizontal2 };

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new DcMotor[] { front_right, back_right, front_left, back_left };

        intake = new DcMotor[] { hardwareMap.dcMotor.get("il"), hardwareMap.dcMotor.get("ir") };

        for (DcMotor motor : intake) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Initialize imu
        logger.addDataUpdate("Status", "Initializing IMU");
        BNO055IMU imuHardware = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(imuHardware);
        imu.initialize();
    }

    protected void initGlobalPosition() {
        globalPositionUpdate = new GlobalPosition(verticalLeft, verticalRight, horizontal, countsPerInch, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }

    protected void intake(float power) {
        for (DcMotor motor : intake) {
            motor.setPower(power);
        }
    }
}