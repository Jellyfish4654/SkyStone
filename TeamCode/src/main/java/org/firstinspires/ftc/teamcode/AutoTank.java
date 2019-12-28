package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Tank;
import org.firstinspires.ftc.teamcode.framework.AutoOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;

import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.enums.SkyStonePosition;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AutoTank")
public class AutoTank extends AutoOpMode {
    DriveTrain drive;

    final double countsPerMM = 500; // change this

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addData("auto status", "Loading AutoTank");
        initHardware();
       // initVision();

        logger.addData("auto status", "Initializing Tank Drivetrain");
        drive = new Tank(Arrays.asList(motors), imu, logger);
        drive.resetEncoders();

        //StonePosition stonePosition = getStonePosition();
        logger.addData("auto status", "Initialization Complete, Waiting for Start");

        waitForStart();
        // ****START****

        // End
        while (opModeIsActive()) {
            idle();
        }
    }

  /*  public void initVision() {
        logger.addData("auto status", "Initializing Vision");
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
  }*/ 
}
