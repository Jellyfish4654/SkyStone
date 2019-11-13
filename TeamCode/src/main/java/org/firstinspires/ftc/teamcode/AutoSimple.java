package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;

import java.util.ArrayList;

@Autonomous(name = "AutoSimple")
public class AutoSimple extends LinearOpMode {
    TFStoneDetector stoneDetector;
    DcMotor mFR, mBR, mFL, mBL; // Declares motors
    DcMotor eRightFront, eRightBack, eLeftFront, eLeftBack; // Declares motor encoders

    ArrayList motors, encoders;

    IIMU imu;
    BNO055IMU boschIMU;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initIMU(0);
        initVision();

        status("Initialization Complete, Waiting for Start");
        waitForStart();
        // ****START****

    }

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
        eRightFront = hardwareMap.dcMotor.get("fr");
        eRightBack = hardwareMap.dcMotor.get("br");
        eLeftFront = hardwareMap.dcMotor.get("fl");
        eLeftBack = hardwareMap.dcMotor.get("bl");

        encoders = new ArrayList<>();
        encoders.add(eRightFront);
        encoders.add(eRightBack);
        encoders.add(eLeftFront);
        encoders.add(eLeftBack);

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
        imu = new IMU(boschIMU);
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
