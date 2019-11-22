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

    public StonePosition getStonePosition() {
        logger.addData("stone status", "Declaring Stone Positions");
        stoneDetector.activateTF();
        logger.addData("stone event", "TensorFlow Activated");
        logger.update();

        List<Recognition> updatedRecognitions = stoneDetector.detectStone();
        boolean validDetectionStatus = false;
        timer.reset();

        while (!validDetectionStatus && timer.milliseconds() < 500 && !isStopRequested()) {
            updatedRecognitions = stoneDetector.detectStone();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2)
                    validDetectionStatus = true;
                else
                    validDetectionStatus = false;
            } else
                validDetectionStatus = false;
            if (timer.seconds() > 5)
                break;
        }

        StonePosition stonePosition = null;

        if (validDetectionStatus){
            logger.addData("stone event", "Valid Detection Confirmed");
            logger.update();
            for(int i = 0; updatedRecognitions.size()>i; i++){
                if(updatedRecognitions.get(i).getLabel()=="Skystone"){
                    logger.addData("stone status", "right edge %f", updatedRecognitions.get(i).getRight());
                    if (updatedRecognitions.get(i).getRight()>=500){ // change this value
                        stonePosition = StonePosition.RIGHT;
                        logger.addData("stone status", "Stone = RIGHT");
                    } else {
                        stonePosition = StonePosition.CENTER;
                        logger.addData("stone status", "Stone = CENTER");
                    }
                }
            }
            if (stonePosition == null) {
                stonePosition = StonePosition.LEFT;
                logger.addData("stone status", "Stone = LEFT");
            }
        } else {
            logger.addData("stone status", "Invalid stone detection");
            logger.update();
        }

        return stonePosition;

        // Reset origin, Check position one, if can be determined return
        // DO NOT RESET ORIGIN Check position two, if can be determined return
        // DO NOT RESET ORIGIN Check position three, if can be determined return
        // DO NOT RESET ORIGIN Move to last pos and return
    }

    public void initVision() {
        logger.addData("auto status", "Initializing Vision");
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
    }
}
