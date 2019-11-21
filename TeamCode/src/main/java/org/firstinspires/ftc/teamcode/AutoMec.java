package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;

import java.util.Arrays;

@Autonomous(name = "AutoMec")
public class AutoMec extends BaseOpMode {
    IDriveTrain drive;
    TFStoneDetector stoneDetector;

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

        // Init Drivetrain Systems and IMU Params
        drive = new Mecanum(Arrays.asList(motors), imu, telemetry, Arrays.asList(motors));
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

    public void initVision() {
        // Init Stone Detector
        stoneDetector = new TFStoneDetector();
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod(0.55);
    }

    // Utility
    public void status(String message) {
        telemetry.addData("Status", message);
        telemetry.update();
    }
}
