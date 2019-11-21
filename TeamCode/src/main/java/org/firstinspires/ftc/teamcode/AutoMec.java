package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
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
    IDriveTrain drive;
    TFStoneDetector stoneDetector;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    final double defaultMaxPower = .9;
    final double defaultMinPower = .3;
    final double defaultMinPowerPivot = .15;

    final double defaultRampUpModifier = .1;
    final double defaultRampDownModifer = .8;
    final double defaultRampDownEndModifer = .9;
    final double defaultErrorDistance = 10;
    final double defaultCorrectionTime = 500; // ms

    final double[] defaultPIDGain = { .02 };
    final double countsPerMM = 500; // change this

    int stonePosition = StonePosition.LEFT; //This is the default if no stones are detected, or if neither of the stones detected are not skystones

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addData("Status - ", "Loading AutoMec");
        initHardware();
        initVision();

        // Init Drivetrain Systems and IMU Params
        logger.addData("Status - ", "Initializing Mecanum Drivetrain");
        drive = new Mecanum(Arrays.asList(motors), imu, telemetry, Arrays.asList(motors));
        drive.resetEncoders();
       
        declareStonePositions();
        logger.addData("Status - ", "Initialization Complete, Waiting for Start");
        waitForStart();
        // ****START****

        // move to stones

        // collect and move to drop

        // move to stone position

    }

    public void declareStonePositions() {
        logger.addData("Status - ", "Declaring Stone Positions");
        stoneDetector.activateTF();
        logger.addData("Event - ", "TensorFlow Activated");
        logger.update();

        List<Recognition> updatedRecognitions = stoneDetector.detectStone();
        boolean validDetectionStatus = false;
        timer.reset();
        timer2.reset();

        while (!validDetectionStatus && timer.milliseconds() < defaultCorrectionTime && !isStopRequested()) {
            updatedRecognitions = stoneDetector.detectStone();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2)
                    validDetectionStatus = true;
                else
                    validDetectionStatus = false;
            } else
                validDetectionStatus = false;
            if (timer2.seconds() > 5)
                break;
        }

        if (validDetectionStatus){
            logger.addData("Event - ", "Valid Detection Confirmed");
            logger.update();
            for(int i = 0; updatedRecognitions.size()>i; i++){
                if(updatedRecognitions.get(i).getLabel()=="Skystone"){
                    logger.addData("SkyStonePosition RightEdge - ", updatedRecognitions.get(i).getRight());
                    if(updatedRecognitions.get(i).getRight()>=500){ // change this value
                        stonePosition = StonePosition.RIGHT;
                        logger.addData("StonePosition - ", "RIGHT");
                    }
                    else
                        stonePosition = StonePosition.CENTER;
                        logger.addData("StonePosition = ", "CENTER");
                }
            }
        }
        else{
            logger.addData("Event - ", "Invalid Detection");
            logger.update();
        }

        // Reset origin, Check position one, if can be determined return
        // DO NOT RESET ORIGIN Check position two, if can be determined return
        // DO NOT RESET ORIGIN Check position three, if can be determined return
        // DO NOT RESET ORIGIN Move to last pos and return
    }

    public void initVision() {
        logger.addData("Status - ", "Initializing Vision");
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
    }
}
