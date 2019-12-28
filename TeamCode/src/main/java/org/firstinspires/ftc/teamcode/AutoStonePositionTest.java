package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.AutoOpMode;
import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;

import org.firstinspires.ftc.teamcode.framework.enums.SkyStonePosition;
import org.firstinspires.ftc.teamcode.framework.enums.Team;
import org.firstinspires.ftc.teamcode.framework.enums.Side;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "StoePosition Test")
public class AutoStonePositionTest extends AutoOpMode {
    DriveTrain drive;
    DriveTrain.MoveParams params;

    final double countsPerMM = 60 * Math.PI * 8192;
    final double countsPerInch = countsPerMM / 25.4;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Status", "Loading AutoMec");
        initHardware();
        initVision(0.55);

        // Init Drivetrain Systems and IMU Params
        logger.addDataUpdate("Status", "Initializing Mecanum Drivetrain");
        drive = new Mecanum(Arrays.asList(motors), imu, logger, Arrays.asList(motors));
        drive.resetEncoders();

        logger.addDataUpdate("Status", "Activating Tensor Flow");
        stoneDetector.activateTF();

        logger.addDataUpdate("Status", "Initialization Complete, Awaiting Settings Override");

        while (!opModeIsActive() && !isStopRequested()) {
            updatedRecognitions = stoneDetector.detectStoneSilent();
            team = gamepad1.dpad_up ? team.RED : (gamepad1.dpad_down ? team.BLUE : team);
            side = gamepad1.dpad_left ? side.STONE : (gamepad1.dpad_left ? side.FOUNDATION : side);

            telemetry.addData("Team", team);
            telemetry.addData("Side", side);
            telemetry.update();
        }

        waitForStart();
        // ****START****
        logger.addDataUpdate("Status", "AutoMec Start - " + team + " Team, " + side + " Side");

        getStonePositions();
        // move to stones
        // collect and move to drop
        // move to stone position

        // End
        while (opModeIsActive()) {
            idle();
        }
    }

}
