package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.enums.StonePosition;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AutoMec")
public class AutoMec extends BaseOpMode {
    DriveTrain drive;
    TFStoneDetector stoneDetector;
    ElapsedTime timer = new ElapsedTime();

    final double countsPerMM = 500; // change this

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addData("auto status", "Loading AutoMec");
        initHardware();
        initVision();

        // Init Drivetrain Systems and IMU Params
        logger.addData("auto status", "Initializing Mecanum Drivetrain");
        drive = new Mecanum(Arrays.asList(motors), imu, logger, Arrays.asList(motors));
        drive.resetEncoders();

        StonePosition stonePosition = getStonePosition();
        logger.addData("auto status", "Initialization Complete, Waiting for Start");
        waitForStart();

        // ****START****
        // move to stones
        // collect and move to drop
        // move to stone position

        while(opModeIsActive()) {
            idle();
        }
    }


    public void initVision() {
        logger.addData("auto status", "Initializing Vision");
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
    }
}
