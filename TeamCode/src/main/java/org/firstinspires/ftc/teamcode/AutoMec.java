package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

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
            intake((float) 1);
            if (team == team.RED) {

                switch (skyStonePosition) {
                case LEFT:

                    break;
                case CENTER:
                    // goToPosition(redStoneCENTER[0][0],redStoneCENTER[0][1]);
                    break;
                default:

                    break;
                }

                // move to stones
                // collect and move to drop
                // move to stone position
                waitMilliseconds(3000, timer);
            } else if (team == team.BLUE) {

                switch (skyStonePosition) {
                case LEFT:

                    break;
                case CENTER:

                    break;
                default:

                    break;
                }

                waitMilliseconds(3000, timer);
            }
        } else if (side == side.FOUNDATION) {
            stoneDetector.shutdownTF();

            waitMilliseconds(25000, timer);

            if (team == team.RED) {
                moveParams = new DriveTrain.MoveParams(30 * countsPerInch, 0, 0, defaultParams);
                resetCalibratedPosition();
                while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
                }
                drive.stop();
            } else if (team == team.BLUE) {
                moveParams = new DriveTrain.MoveParams(30 * countsPerInch, 0, 0, defaultParams);
                resetCalibratedPosition();
                while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
                }
                drive.stop();
            }
        }
        // End
        intake(0);
        while (opModeIsActive()) {
            drive.stop();
            ReadWriteFile.writeFile(autoIMUOffset, String.valueOf(imu.getZAngle() - 45));
            telemetry.addData("Status", "Auto Complete, Idle Mode");
            telemetry.update();
        }
    }

    public void moveCalibratedTest() {
        moveParams = new DriveTrain.MoveParams(5 * countsPerInch, 0, 0, defaultParams);
        moveParams.debugMove = true;

        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && !isStopRequested()) {
        }
        drive.stop();

        // Testing for continuous motion with turn
        waitMilliseconds(3000, timer);
        moveParams = new DriveTrain.MoveParams(20 * countsPerInch, 65, -70, defaultParams);
        resetCalibratedPosition();
        waitMilliseconds(3000, timer);
        logger.addDataUpdate("Encoder distance reset check", calibratedCurrentPosition());
        waitMilliseconds(3000, timer);
        while (drive.move(calibratedCurrentPosition(), moveParams) && !isStopRequested()) {
            telemetry.addData("Status", "Executing trial movement 2");
            telemetry.addData("Distance To Target", (moveParams.targetPosition / countsPerInch)
                    - (Math.abs(calibratedCurrentPosition()) / countsPerInch));
            telemetry.update();
        }
        drive.stop();
    }

    public void pivotTest() {
        pivotParams = new DriveTrain.PivotParams(90, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        waitMilliseconds(3000, timer);

        pivotParams = new DriveTrain.PivotParams(180, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        waitMilliseconds(3000, timer);

        pivotParams = new DriveTrain.PivotParams(270, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        waitMilliseconds(3000, timer);

        pivotParams = new DriveTrain.PivotParams(180, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        waitMilliseconds(3000, timer);

        pivotParams = new DriveTrain.PivotParams(0, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();
    }

    public void blueSideFoundation() {
        foundationLeft.setPosition(foundationLeftRetract);
        foundationRight.setPosition(foundationRightRetract);

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, 0, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();

        foundationLeft.setPosition(foundationLeftExtend);
        foundationRight.setPosition(foundationRightExtend);

        pivotParams = new DriveTrain.PivotParams(-90, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, -67.5, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();

        foundationLeft.setPosition(foundationLeftRetract);
        foundationRight.setPosition(foundationRightRetract);

        while(goToPosition(0, 0, 90, 1, .35) && opModeIsActive());
        drive.stop();

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, 0, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();

    }

    public void redSideFoundation() {
        foundationLeft.setPosition(foundationLeftRetract);
        foundationRight.setPosition(foundationRightRetract);

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, 0, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();

        foundationLeft.setPosition(foundationLeftExtend);
        foundationRight.setPosition(foundationRightExtend);

        pivotParams = new DriveTrain.PivotParams(90, defaultParams);
        while (drive.pivot(pivotParams) && opModeIsActive())
            ;
        drive.stop();

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, 67.5, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();

        foundationLeft.setPosition(foundationLeftRetract);
        foundationRight.setPosition(foundationRightRetract);

        while(goToPosition(0,0,-90,1,.35)&& opModeIsActive());
        drive.stop();

        moveParams = new DriveTrain.MoveParams(24 * countsPerInch, 0, 0, defaultParams);
        resetCalibratedPosition();
        while (drive.move(calibratedCurrentPosition(), moveParams) && opModeIsActive()) {
        }
        drive.stop();
    }

}
