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
import org.firstinspires.ftc.teamcode.logging.FileLogger;

import java.util.Calendar;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor front_right, back_right, front_left, back_left;
    protected DcMotor verticalLeft, verticalRight, horizontal, horizontal2;

    protected DcMotor[] motors, encoders;
    protected DcMotor[] intake;

    protected IMU imu;
    protected GlobalPosition globalPositionUpdate;
    protected Thread positionThread;

    protected DoubleLogger logger = new DoubleLogger(telemetry);

    protected ElapsedTime timer = new ElapsedTime();
    protected ElapsedTime debugTimer = new ElapsedTime();
    protected Calendar now = Calendar.getInstance();

    protected DebugMode debugMode = DebugMode.NONE;
    protected String lastPositionSave = "N/A";

    protected final double countsPerMM = 8192 / (60 * Math.PI); // 8192
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

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Just a test to // see if all the sperate encoder reversals are even needed.
         * Test to see if the back_right motor is affected.
         */
        // verticalRight.setDirection(DCMotorSimple.Direction.REVERSE);

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

        intake = new DcMotor[] { hardwareMap.dcMotor.get("il"), hardwareMap.dcMotor.get("ir") }; // left 1, right 0

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
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    protected void intake(float power) {
        for (DcMotor motor : intake) {
            motor.setPower(power);
        }
    }

    protected void positionTelemetry() {
        telemetry.addData("Last Position Save", lastPositionSave);

        if (positionThread.isAlive()) {
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / countsPerInch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            // Should be already software coded to be reversed in all instances.
            telemetry.addData("Vertical right encoder position", -verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal encoder position", horizontal.getCurrentPosition());
        }

        telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.update();
    }

    protected void positionSave() {
        if (debugTimer.seconds() < 2 | debugMode != DebugMode.ALL) {
        } else {
            lastPositionSave = getTime();

            if (positionThread.isAlive()) {
                FileLogger.addData("X Position", globalPositionUpdate.returnXCoordinate() / countsPerInch);
                FileLogger.addData("Y Position", globalPositionUpdate.returnYCoordinate() / countsPerInch);
                FileLogger.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                FileLogger.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                // Should be already software coded to be reversed in all instances.
                FileLogger.addData("Vertical right encoder position", -verticalRight.getCurrentPosition());
                FileLogger.addData("Horizontal encoder position", horizontal.getCurrentPosition());
            }

            FileLogger.addData("Thread Active", positionThread.isAlive());

            debugTimer.reset();
        }
    }

    protected String getTime() {
        return String.format("%2d:%2d:%2d ", now.get(Calendar.HOUR_OF_DAY), now.get(Calendar.MINUTE),
                now.get(Calendar.SECOND));
    }

}