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
import java.util.ArrayList;

@Autonomous(name = "AutoMec")
public class AutoMec extends AutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Status", "Loading AutoMec");
        initHardware();
        initVision(0.55);
        // Init Drivetrain Systems and IMU Params
        initMecanum();

        logger.addDataUpdate("Status", "Activating Tensor Flow");
        stoneDetector.activateTF();

        logger.addDataUpdate("Status", "Initialization Complete, Awaiting Settings Override");

        while (!opModeIsActive() && !isStopRequested()) {
            updatedRecognitions = stoneDetector.detectStoneSilent();
            team = gamepad1.dpad_up ? team.RED : (gamepad1.dpad_down ? team.BLUE : team);
            side = gamepad1.dpad_left ? side.STONE : (gamepad1.dpad_right ? side.FOUNDATION : side);
            telemetry.addData("Status", "Initialization Complete, Awaiting Settings Override");

            telemetry.addData("Team", team);
            telemetry.addData("Side", side);
            telemetry.update();
        }

        // ***START***//
        waitForStart();
        timer.reset();
        drive.resetEncoders();
        initGlobalPosition();
        logger.addDataUpdate("Status", "AutoMec Start - " + team + " Team, " + side + " Side");

        if (side == side.STONE) {
            getStonePositions();
            stoneDetector.shutdownTF();

            intake((float) 0.75);

            if (team == team.RED) {

                switch (skyStonePosition) {
                case LEFT:

                    break;
                case CENTER:

                    break;
                default:

                    break;
                }

                // move to stones
                // collect and move to drop
                // move to stone position

                moveTest();
            } else if (team == team.BLUE) {

                switch (skyStonePosition) {
                case LEFT:

                    break;
                case CENTER:

                    break;
                default:

                    break;
                }
            }
        } else if (side == side.FOUNDATION) {
            stoneDetector.shutdownTF();

            while (timer.seconds() < 25)
                ;

            if (team == team.RED) {
                moveTest();

            } else if (team == team.BLUE) {
                moveTest();
            }
        }
        // End
        while (opModeIsActive()) {
            idle();
        }
    }

    public void moveTest() {
        params = new DriveTrain.MoveParams(5 * countsPerInch, 0, 0, defaultParams);
        params.debugMove = true;
       
        drive.softEncoderReset();
        while (drive.move(drive.getEncoderDistance(), params) && !isStopRequested()) {
            // logger.addData("Data",drive.data());
        }
        // Testing for continuous motion with turn
        /*
         * params = new DriveTrain.MoveParams(24 * countsPerInch, 20, 10);
         * drive.softEncoderReset(); while (drive.move(drive.getEncoderDistance(),
         * params) & drive.getEncoderDistance() / countsPerInch < 12) ; params.moveAngle
         * = 50; while (drive.move(drive.getEncoderDistance(), params)) ;
         */
    }

}
