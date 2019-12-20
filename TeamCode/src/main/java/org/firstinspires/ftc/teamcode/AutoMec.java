package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.AutoOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.enums.StonePosition;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AutoMec")
public class AutoMec extends AutoOpMode {
    DriveTrain drive;
    DriveTrain.MoveParams params;

    final double countsPerMM = 60 * Math.PI * 8192;
    final double countsPerInch = countsPerMM / 25.4;

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

        params = new DriveTrain.MoveParams(24 * countsPerInch, 20, 10);
        drive.softEncoderReset();
        while (drive.move(drive.getEncoderDistance(), params))
            ;

        // Testing for continuous motion with turn
        params = new DriveTrain.MoveParams(24 * countsPerInch, 20, 10);
        drive.softEncoderReset();
        while (drive.move(drive.getEncoderDistance(), params) & drive.getEncoderDistance() / countsPerInch < 12)
            ;
        params.moveAngle = 50;
        while (drive.move(drive.getEncoderDistance(), params))
            ;
        // End
        while (opModeIsActive()) {
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
