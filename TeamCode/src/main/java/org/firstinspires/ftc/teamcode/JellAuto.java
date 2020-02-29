package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.GlobalPosition;

import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.framework.enums.Motors;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;

import java.util.List;
import java.util.Arrays;

@Autonomous(name = "JellAuto")
public final class JellAuto extends BaseOpMode {
    private static enum Team {
        RED, BLUE
    }
    private static enum Mode {
        PARK, FOUNDATION,
        STONE, STONEPLACE
    }
    private static enum Skystone {
        LEFT, RIGHT, CENTER
    }

    private Team team;
    private Mode mode;
    private Skystone skystonePosition;

    private DriveTrain drivetrain;
    private DriveTrain.DefaultParams defaultParams;
    private void initDrivetrain() {
        drivetrain = new Mecanum(Arrays.asList(motors), imu, logger, Arrays.asList(encoders));
        defaultParams = new DriveTrain.DefaultParams(
            0.9, 0.35, // max, min
            0 * countsPerInch, 6 * countsPerInch, 12 * countsPerInch, // ramp {up,down,downend}
            new double[] {.03, .03, .03 }, // pid
            350, .2 * countsPerInch, Direction.FASTEST);
    }

    private TFStoneDetector stoneDetector = new TFStoneDetector();
    private void initVision(double confidence) {
        stoneDetector.initVuforia(this, "webcam");
        stoneDetector.initTfod(confidence); // 0.55?
        logger.addDataUpdate("status", "Finished vision initialization");
    }

    private Skystone getSkystone() {
        List<Recognition> updatedRecognitions = stoneDetector.detectStone();

        // sort stones, right to left
        try {
            for (int i = 0; updatedRecognitions.size() > i; i++) {
                if (updatedRecognitions.get(i).getRight() > updatedRecognitions.get(0).getRight()) {
                    Recognition temp = updatedRecognitions.get(0);
                    updatedRecognitions.set(0, updatedRecognitions.get(i));
                    updatedRecognitions.set(i, temp);
                }
            }

            // check for skystones, set default based on Team
            if (team == team.RED) {
                for (int i = 0; updatedRecognitions.size() > i; i++) {
                    if (updatedRecognitions.get(i).getLabel() == "Skystone") {
                        switch (i) {
                        case 0:
                            return Skystone.RIGHT;
                        case 1:
                            return Skystone.CENTER;
                        case 2:
                            return Skystone.LEFT;
                        }
                    }
                }
            } else if (team == team.BLUE) {
                for (int i = 0; updatedRecognitions.size() > i; i++) {
                    if (updatedRecognitions.get(i).getLabel() == "Skystone") {
                        switch (i) {
                        case 0:
                            return Skystone.LEFT;
                        case 1:
                            return Skystone.CENTER;
                        case 2:
                            return Skystone.RIGHT;
                        }
                    }
                }
            }

            logger.addData("skystone", "No stone detected");
            logger.update();
            return null;
        } catch (NullPointerException nullPointer) {
            logger.addData("skystone", "No stone detected");
            logger.update();
            return null;
        }
    }

    /*
    protected boolean goToPosition(double targetX, double targetY, double targetOrientation, double maxPower,
            double minPower) {
        double xDistance = targetX - globalPositionUpdate.returnXCoordinate();
        double yDistance = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        double robotOrientationDifference = targetOrientation - imu.getZAngle();

        double power;
        if (distance > defaultRampDownPos) {
            power = maxPower;
        } else if (distance < defaultRampDownEndPos) {
            power = minPower;
        } else {
            double rampDownPositionDifference = defaultRampDownPos - defaultRampDownEndPos;
            double distanceRampDownDifference = distance - defaultRampDownEndPos;
            power = -((minPower - maxPower) / (rampDownPositionDifference)) * (distanceRampDownDifference) + minPower;
        }

        double moveAngle;
        moveAngle = Math.toDegrees(Math.atan(xDistance / yDistance));
        if ((xDistance < 0 && yDistance < 0) || (xDistance > 0 && yDistance < 0)) {
            moveAngle += 180;
        }
        moveAngle = (moveAngle % 360);

        if (!(Math.abs(yDistance) < .75 * countsPerInch && Math.abs(xDistance) < .75 * countsPerInch
                && Math.abs(robotOrientationDifference) < 5)) {
            moveParams = new DriveTrain.MoveParams(distance, moveAngle, targetOrientation, defaultParams);

            moveParams.maxPower = power;
            moveParams.minPower = power;
            moveParams.rampDown = distance;
            moveParams.rampUp = 0;
            moveParams.rampDownEnd = distance;

            drivetrain.move(0, moveParams);

            telemetry.addData("Moving to Position", targetX + ", " + targetY);
            telemetry.addData("Distance to Target", distance / countsPerInch);
            telemetry.addData("X Distance to Target", xDistance / countsPerInch);
            telemetry.addData("Y Distance to Target", yDistance / countsPerInch);
            telemetry.addData("Move Angle", moveAngle);
            telemetry.update();
            return true;
        } else {
            return false;
        }
    }
    */

    // Global Positioning
    protected GlobalPosition globalPositionUpdate;
    protected Thread positionThread;
    protected double xOffset;
    protected double yOffset;
    protected double getCalPos() {
        double xDistance = globalPositionUpdate.returnXCoordinate() - xOffset;
        double yDistance = globalPositionUpdate.returnYCoordinate() - yOffset;

        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }

    protected void resetCalPos() {
        xOffset = globalPositionUpdate.returnXCoordinate();
        yOffset = globalPositionUpdate.returnYCoordinate();
    }

    protected void initGlobalPosition() {
        globalPositionUpdate = new GlobalPosition(encoders[Motors.E_VL], encoders[Motors.E_VR], encoders[Motors.E_H], countsPerInch, 70);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        //globalPositionUpdate.reverseRightEncoder();
    }

    /*
    protected void positionTelemetry() {
        telemetry.addData("Last Position Save", lastPositionSave);

        if (positionThread.isAlive()) {
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / countsPerInch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", -verticalLeft.getCurrentPosition());
            // Should be already software coded to be reversed in all instances.
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal encoder position", horizontal.getCurrentPosition());
        }
        telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.addData("Lift position", lift.getCurrentPosition());
    }

    protected void positionSave() {
        if (debugTimer.seconds() < 2 | debugMode != DebugMode.ALL) {
        } else {
            lastPositionSave = Utility.getTime();

            if (positionThread.isAlive()) {
                FileLogger.addData("X Position", globalPositionUpdate.returnXCoordinate() / countsPerInch);
                FileLogger.addData("Y Position", globalPositionUpdate.returnYCoordinate() / countsPerInch);
                FileLogger.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                FileLogger.addData("Vertical left encoder position", -verticalLeft.getCurrentPosition());
                // Should be already software coded to be reversed in all instances.
                FileLogger.addData("Vertical right encoder position", -verticalRight.getCurrentPosition());
                FileLogger.addData("Horizontal encoder position", -horizontal.getCurrentPosition());
            }

            FileLogger.addData("Thread Active", positionThread.isAlive());

            debugTimer.reset();
        }
    } */
    
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initDrivetrain();
//        initVision(0.55);

        while (!opModeIsActive() && !isStopRequested()) { 
            // maybe detectStoneSilent?
            if (gamepad1.x)
                team = Team.BLUE;
            if (gamepad1.b)
                team = Team.RED;
            
            if (gamepad1.dpad_up)
                mode = Mode.PARK;
            if (gamepad1.dpad_left)
                mode = Mode.FOUNDATION;
            if (gamepad1.dpad_down)
                mode = Mode.STONE;
            if (gamepad1.dpad_right)
                mode = Mode.STONEPLACE;

            telemetry.addData("team", team);
            telemetry.addData("mode", mode);
        }

        waitForStart();
        drivetrain.resetEncoders();
        initGlobalPosition();

        skystonePosition = getSkystone();
        stoneDetector.shutdownTF();

        switch (mode) {
            case PARK:
                park();
                break;
            case FOUNDATION:
                foundation();
                break;
            default:
                telemetry.addData("error", "hi, you're screwed if this is comp");
                telemetry.update();
        }
    }

    private void park() {
        sleep(22000);

        DriveTrain.MoveParams moveParams = new DriveTrain.MoveParams(36 * countsPerInch, 0, 0, defaultParams);
        resetCalPos();
        while (drivetrain.move(getCalPos(), moveParams) && opModeIsActive());
        drivetrain.stop();
    }

    private void foundation() {
        DriveTrain.MoveParams moveParams;

        moveParams = new DriveTrain.MoveParams(30 * countsPerInch, 180, 0, defaultParams);
        resetCalPos();
        while (drivetrain.move(getCalPos(), moveParams) && opModeIsActive());
        drivetrain.stop();

        foundation.extend();
        sleep(1000);

        moveParams = new DriveTrain.MoveParams(30 * countsPerInch, 0, 0, defaultParams);
        resetCalPos();
        while (drivetrain.move(getCalPos(), moveParams) && opModeIsActive());
        drivetrain.stop();

        foundation.retract();
        sleep(1000);

        moveParams = new DriveTrain.MoveParams(36 * countsPerInch, 270, 0, defaultParams);
        resetCalPos();
        while (drivetrain.move(getCalPos(), moveParams) && opModeIsActive());
        drivetrain.stop();
    }
}
