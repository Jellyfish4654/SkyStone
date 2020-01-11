package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.subsystems.GlobalPosition;
import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;
import org.firstinspires.ftc.teamcode.logging.FileLogger;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.Calendar;
import java.io.File;

public abstract class BaseOpMode extends LinearOpMode {
    protected DcMotor front_right, back_right, front_left, back_left;
    protected DcMotor verticalLeft, verticalRight, horizontal, horizontal2;

    protected Servo foundationLeft, foundationRight;
    protected Servo stoneIntake, stoneOutake;

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

    protected final double foundationLeftRetract = .58;
    protected final double foundationLeftExtend = .1;

    protected final double foundationRightRetract = .08;
    protected final double foundationRightExtend = .56;

    protected final double stoneIntakeOpen = 0.77; // wrong
    protected final double stoneIntakeLock = .5; // values
    protected final double stoneIntakeExtend = .15; // here!

    protected final double stoneOutputLock = 0;
    protected final double stoneOutputOpen = .5; // likely wrong

    protected void initGlobalPosition() {
        globalPositionUpdate = new GlobalPosition(verticalLeft, verticalRight, horizontal, countsPerInch, 70);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
    }

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

        foundationLeft = hardwareMap.servo.get("lFoundation"); // 3
        foundationRight = hardwareMap.servo.get("rFoundation"); // 2
        stoneIntake = hardwareMap.servo.get("sI"); // 0
        stoneOutake = hardwareMap.servo.get("sO"); // 1

        // front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        // back_right.setDirection(DcMotorSimple.Direction.REVERSE);

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

    protected void intake(float power) {
        for (DcMotor motor : intake) {
            motor.setPower(power);
        }
    }

    protected void output() {
        stoneOutake.setPosition(stoneOutputOpen);
        stoneIntake.setPosition(stoneIntakeExtend);
    }

    protected void foundation() {
        if (foundationLeft.getPosition() != foundationLeftExtend) {
            foundationLeft.setPosition(foundationLeftExtend);
            foundationRight.setPosition(foundationRightExtend);
        } else {
            foundationLeft.setPosition(foundationLeftRetract);
            foundationRight.setPosition(foundationRightRetract);
        }
    }

    protected void positionTelemetry() {
        telemetry.addData("Last Position Save", lastPositionSave);

        if (positionThread.isAlive()) {
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / countsPerInch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", -verticalLeft.getCurrentPosition());
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
                FileLogger.addData("Vertical left encoder position", -verticalLeft.getCurrentPosition());
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

    protected void waitMilliseconds(double milliseconds, ElapsedTime timer) {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds)
            ;
    }

}