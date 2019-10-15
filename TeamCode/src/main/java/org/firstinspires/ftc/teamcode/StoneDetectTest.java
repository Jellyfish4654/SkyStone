package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.Datalog;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TFStone: Working proof of concept", group = "Concept")
public class StoneDetectTest extends LinearOpMode {

    TFStoneDetector stoneDetector = new TFStoneDetector();

    public void runOpMode() {
        stoneDetector.initVuforia(this);
        stoneDetector.initTfod();
        stoneDetector.activateTF();

        waitForStart();

        while (opModeIsActive()) {
            stoneDetector.detectStone();
        }
        stoneDetector.shutdownTF();
    }
}