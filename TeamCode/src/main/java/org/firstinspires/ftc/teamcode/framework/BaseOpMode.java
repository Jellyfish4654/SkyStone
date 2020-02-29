package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.subsystems.GlobalPosition;
import org.firstinspires.ftc.teamcode.framework.enums.Motors;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;
import org.firstinspires.ftc.teamcode.logging.FileLogger;

import org.firstinspires.ftc.teamcode.framework.subsystems.Foundation;
import org.firstinspires.ftc.teamcode.framework.subsystems.Intake;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor[] motors, encoders;

    protected IMU imu;
//    protected RevBlinkinLedDriver led;

    protected DoubleLogger logger = new DoubleLogger(telemetry);
    protected ElapsedTime timer = new ElapsedTime();
    protected ElapsedTime debugTimer = new ElapsedTime();
    protected DebugMode debugMode = DebugMode.NONE;
    protected String lastPositionSave = "N/A";

    protected final double countsPerMM = 8192 / (60 * Math.PI); // 8192
    protected final double countsPerInch = countsPerMM * 25.4;

/* REMOVE
    protected File autoIMUOffset = AppUtil.getInstance().getSettingsFile("autoIMUOffset");
    protected File liftEncoderPosition = AppUtil.getInstance().getSettingsFile("liftEncoderPosition.txt");

    protected File redStoneLEFTPositions = AppUtil.getInstance().getSettingsFile("redStoneLEFTPositions.txt");
    protected File redStoneCENTERPositions = AppUtil.getInstance().getSettingsFile("redStoneCENTERPositions.txt");
    protected File redStoneRIGHTPositions = AppUtil.getInstance().getSettingsFile("redStoneRIGHTPositions.txt");
    protected File redFoundationPositions = AppUtil.getInstance().getSettingsFile("redFoundationPositions.txt");

    protected File blueStoneLEFTPositions = AppUtil.getInstance().getSettingsFile("blueStoneLEFTPositions.txt");
    protected File blueStoneCENTERPositions = AppUtil.getInstance().getSettingsFile("blueStoneCENTERPositions.txt");
    protected File blueStoneRIGHTPositions = AppUtil.getInstance().getSettingsFile("blueStoneRIGHTPositions.txt");
    protected File blueFoundationPositions = AppUtil.getInstance().getSettingsFile("blueFoundationPositions.txt");

    protected int X_POS_INDEX = 0;
    protected int Y_POSTINDEX = 1;
    protected int THETA_INDEX = 2;
    protected int MAX_POWER_INDEX = 3;
    protected int MIN_POWER_INDEX = 4;
*/

    protected Foundation foundation;
    protected Intake intake;

    protected void initHardware() {
        // Initialize drivetrain
        DcMotor motorFR = hardwareMap.dcMotor.get("fr"); // 0
        DcMotor motorBR = hardwareMap.dcMotor.get("br"); // 1
        DcMotor motorFL = hardwareMap.dcMotor.get("fl"); // 2
        DcMotor motorBL = hardwareMap.dcMotor.get("bl"); // 3
        motors = new DcMotor[4];
        motors[Motors.FR] = motorFR;
        motors[Motors.BR] = motorBR;
        motors[Motors.FL] = motorFL;
        motors[Motors.BL] = motorBL;

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize odometry
        DcMotor encoderVL = hardwareMap.dcMotor.get("il"); // 0 **SHARES INTAKE**
        DcMotor encoderVR = hardwareMap.dcMotor.get("ir"); // 1 **SHARES INTAKE**
        DcMotor encoderH = hardwareMap.dcMotor.get("eH");// 2 **BY ITSELF**
        encoders = new DcMotor[3];
        encoders[Motors.E_VL] = encoderVL;
        encoders[Motors.E_VR] = encoderVR;
        encoders[Motors.E_H] = encoderH;

        // Initalize Foundation
        foundation = new Foundation(hardwareMap.servo.get("lFoundation"), hardwareMap.servo.get("rFoundation")); // {3,2}

        // Initalize Intake
/*        DcMotor intakeL = hardwareMap.dcMotor.get("il"); // 0
        DcMotor intakeR = hardwareMap.dcMotor.get("ir"); // 1
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] intakeMotors = new DcMotor[]{intakeL, intakeR};
*/
        DcMotor[] intakeMotors = new DcMotor[0];
        for (DcMotor motor: intakeMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake = new Intake(intakeMotors, null, null, null);

        // Initialize IMU
        BNO055IMU imuHardware = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(imuHardware);
        imu.initialize();

        logger.addDataUpdate("status", "Finished hardware initialization");
    }

    protected void setPower(double[] motorPowers) {
        motors[Motors.FR].setPower(motorPowers[Motors.FR]);
        motors[Motors.BR].setPower(motorPowers[Motors.BR]);
        motors[Motors.FL].setPower(motorPowers[Motors.FL]);
        motors[Motors.BL].setPower(motorPowers[Motors.BL]);
    }

/*
    public void waitMilliseconds(double milliseconds, ElapsedTime timer) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds)
            ;
    }
*/
}
