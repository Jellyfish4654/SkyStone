package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.framework.subsystems.TFStoneDetector;

import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

import org.firstinspires.ftc.teamcode.framework.movement.BaseOpMode;


@Autonomous(name = "AutoSimple")
public class AutoSimple extends BaseOpMode {
    TFStoneDetector stoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initVision();

        status("Initialization Complete, Waiting for Start");
        waitForStart();
        // ****START****
    }

    @Override
    protected void initHardware() {
        super.initHardware();
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
