package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

@Autonomous(name = "AutoMec")
public class AutoMec extends LinearOpMode {
    IDriveTrain drive;
    TFStoneDetector stoneDetector;

    DcMotor mFR, mBR, mFL, mBL; // Declare mecanum motors
    DcMotor eVerticalLeft, eVerticalRight, eHorizontal, eHorizontalEmpty; // Declares odometry encoder (last one is
                                                                          // empty)
    ArrayList motors, encoders;

    IMU imu;
    BNO055IMU boschIMU;

    final double defaultMaxPower = .9;
    final double defaultMinPower = .3;
    final double defaultMinPowerPivot = .15;

    final double defaultRampUpModifier = .1;
    final double defaultRampDownModifer = .8; 
    final double defaultRampDownEndModifer = .9;
    final double defaultErrorDistance = 10;
    final double defaultCorrectionTime = 500; // ms

    final double[] defaultPIDGain = { .02 };
    final double countsPerMM = 500; //change this

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initVision();
        initIMU(0);

        // Init Drivetrain Systems and IMU Params
        drive = new Mecanum(motors, imu, telemetry, encoders);
        drive.resetEncoders();

        status("Initialization Complete, Waiting for Start");
        waitForStart();
        // ****START****

        // move to stones

        declareStonePositions();

        // collect and move to drop

        // move to stone position

    }

    public void declareStonePositions() {
        // Reset origin, Check position one, if can be determined return
        // DO NOT RESET ORIGIN Check position two, if can be determined return
        // DO NOT RESET ORIGIN Check position three, if can be determined return
        // DO NOT RESET ORIGIN Move to last pos and return
    }

    /**
     * Simplification of drive move
     * 
     * @param targetPosition
     * @param moveAngle
     * @param endOrientationAngle
     */
    public void move(boolean resetOrigin, double targetPosition, double moveAngle, double endOrientationAngle) {
        double rampDownTargetPosition = targetPosition * defaultRampDownModifer;
        double rampUpTargetPosition = targetPosition * defaultRampUpModifier;
        double rampDownEnd = targetPosition * defaultRampDownEndModifer;

        if (resetOrigin)
            drive.softEncoderReset();

        while (opModeIsActive() && drive.move(drive.getEncoderDistance(), targetPosition, rampDownTargetPosition,
                rampUpTargetPosition, rampDownEnd, defaultMaxPower, defaultMinPower, moveAngle, defaultPIDGain,
                endOrientationAngle, defaultErrorDistance, defaultCorrectionTime))
            ;
        drive.stop();

    }

    // Initialization steps
    public void initHardware() {
        /*******************************
         **** Init Motors and Encoders***
         *********************************/

        /*
         * Assumes the following hardware map RightFront motor is with vertical left
         * encoder RightBack motor is with vertical right encoder LeftFront motor is
         * with horizontal encoder LeftBack motor is with a dummy encoder
         */

        // Drive Motors
        mFR = hardwareMap.dcMotor.get("fr");
        mBR = hardwareMap.dcMotor.get("br");
        mFL = hardwareMap.dcMotor.get("fl");
        mBL = hardwareMap.dcMotor.get("bl");

        motors = new ArrayList<>();
        motors.add(mFR);
        motors.add(mBR);
        motors.add(mFL);
        motors.add(mBL);

        // Odometry encoders
        eVerticalLeft = hardwareMap.dcMotor.get("fr");
        eVerticalRight = hardwareMap.dcMotor.get("br");
        eHorizontal = hardwareMap.dcMotor.get("fl");
        eHorizontalEmpty = hardwareMap.dcMotor.get("bl");

        encoders = new ArrayList<>();
        encoders.add(eVerticalLeft);
        encoders.add(eVerticalRight);
        encoders.add(eHorizontal);
        encoders.add(eHorizontalEmpty);

        status("Motor and Encoder Hardware Initialized");
    }

    public void initVision() {
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
    }

    /**
     * 
     * @param offSet
     */
    public void initIMU(double offSet) {
        // Init IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BNO055(boschIMU);
        imu.initialize();
        imu.setOffSet(offSet);
        status("IMU Initialized");
    }

    // Utility
    public void status(String message) {
        telemetry.addData("Status", message);
        telemetry.update();
    }
}
