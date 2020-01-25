package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;

import org.firstinspires.ftc.teamcode.framework.enums.SkyStonePosition;
import org.firstinspires.ftc.teamcode.framework.enums.Team;
import org.firstinspires.ftc.teamcode.framework.enums.Side;
import org.firstinspires.ftc.teamcode.framework.enums.Direction;
import org.firstinspires.ftc.teamcode.framework.enums.MoveIndex;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public abstract class AutoOpMode extends BaseOpMode {

    protected List<Recognition> updatedRecognitions;

    protected Team team = Team.RED;
    protected Side side = Side.STONE;
    protected SkyStonePosition skyStonePosition = null;

    protected TFStoneDetector stoneDetector = new TFStoneDetector();

    protected static final String webCam = "Webcam 1";

    protected DriveTrain drive;

    protected DriveTrain.DefaultParams defaultParams;
    protected DriveTrain.MoveParams moveParams;
    protected DriveTrain.PivotParams pivotParams;

    protected final double defaultRampDownPos = 10 * countsPerInch;
    protected final double defaultRampDownEndPos = 3 * countsPerInch;

    protected final double defaultMaxPower = 0.90;
    protected final double defaultMinPower = 0.35;

    protected final double defaultRampUp = 0 * countsPerInch;
    protected final double defaultRampDown = 6 * countsPerInch;
    protected final double defaultRampDownEnd = 12 * countsPerInch;

    protected final double[] defaultPIDGain = { .03, .03, .03 };

    protected final double defaultCorrectionTime = 350; // changed
    protected final double defaultErrorDistance = countsPerInch * .3; // changed

    protected final Direction defaultDirection = Direction.FASTEST;

    protected String fileText;
    protected String[] inputs;

    protected double[][] redStoneLEFT, redStoneCENTER,redStoneRIGHT, redStoneFOUNDATION;

    protected double[][] blueStoneLEFT, blueStoneCENTER, blueStoneRIGHT,blueStoneFOUNDATION;

    protected double xOffset = 0;
    protected double yOffset = 0;

    public void initMecanum() {
        logger.addDataUpdate("Status", "Initializing Mecanum Drivetrain");
        drive = new Mecanum(Arrays.asList(motors), imu, logger, Arrays.asList(encoders));
        defaultParams = new DriveTrain.DefaultParams(defaultMaxPower, defaultMinPower, defaultRampUp, defaultRampDown,
                defaultRampDownEnd, defaultPIDGain, defaultCorrectionTime, defaultErrorDistance, defaultDirection);
    }

    public void initVision(double confidence) {
        logger.addDataUpdate("Status", "Initializing Vision");

        stoneDetector.initVuforia(this, webCam);
        stoneDetector.initTfod(confidence); // 0.55?
    }

    public void readFiles() {
        logger.addDataUpdate("Status", "Reading Red Side, Left Skystone Position File");

        fileText = ReadWriteFile.readFile(redStoneLEFTPositions);
        inputs = fileText.split("~");
        redStoneLEFT = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                redStoneLEFT[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading Red Side, Center Skystone Position File");

        fileText = ReadWriteFile.readFile(redStoneCENTERPositions);
        inputs = fileText.split("~");
        redStoneCENTER = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                redStoneCENTER[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading Red Side, Right Skystone Position File");

        fileText = ReadWriteFile.readFile(redStoneRIGHTPositions);
        inputs = fileText.split("~");
        redStoneRIGHT = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                redStoneRIGHT[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading Red Side, Foundation Side Position File");

        fileText = ReadWriteFile.readFile(redFoundationPositions);
        inputs = fileText.split("~");
        redStoneFOUNDATION = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                redStoneFOUNDATION[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading blue Side, Left Skystone Position File");

        fileText = ReadWriteFile.readFile(blueStoneLEFTPositions);
        inputs = fileText.split("~");
        blueStoneLEFT = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                blueStoneLEFT[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading blue Side, Center Skystone Position File");

        fileText = ReadWriteFile.readFile(blueStoneCENTERPositions);
        inputs = fileText.split("~");
        blueStoneCENTER = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                blueStoneCENTER[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading Blue Side, Right Skystone Position File");

        fileText = ReadWriteFile.readFile(blueStoneRIGHTPositions);
        inputs = fileText.split("~");
        blueStoneRIGHT = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                blueStoneRIGHT[i][j] = Double.parseDouble(params[j]);
            }
        }

        logger.addDataUpdate("Status", "Reading Blue Side, Foundation Side Position File");

        fileText = ReadWriteFile.readFile(blueFoundationPositions);
        inputs = fileText.split("~");
        blueStoneFOUNDATION = new double[inputs.length][5];
        for (int i = 0; i < inputs.length; i++) {
            String[] params = inputs[i].split(",");
            for (int j = 0; j < params.length; j++) {
                blueStoneFOUNDATION[i][j] = Double.parseDouble(params[j]);
            }
        }
    }

    public void getStonePositions() {
        updatedRecognitions = stoneDetector.detectStone();

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
                skyStonePosition = SkyStonePosition.LEFT;
                for (int i = 0; updatedRecognitions.size() > i; i++) {
                    if (updatedRecognitions.get(i).getLabel() == "Skystone") {
                        switch (i) {
                        case 0:
                            skyStonePosition = SkyStonePosition.RIGHT;
                            break;
                        case 1:
                            skyStonePosition = SkyStonePosition.CENTER;
                            break;
                        case 2:
                            skyStonePosition = SkyStonePosition.LEFT;
                            break;
                        default:
                            skyStonePosition = SkyStonePosition.RIGHT;
                            break;
                        }
                    }
                }
            } else if (team == team.BLUE) {
                skyStonePosition = SkyStonePosition.RIGHT;
                for (int i = 0; updatedRecognitions.size() > i; i++) {
                    if (updatedRecognitions.get(i).getLabel() == "Skystone") {
                        switch (i) {
                        case 0:
                            skyStonePosition = SkyStonePosition.LEFT;
                            break;
                        case 1:
                            skyStonePosition = SkyStonePosition.CENTER;
                            break;
                        case 2:
                            skyStonePosition = SkyStonePosition.RIGHT;
                            break;
                        default:
                            skyStonePosition = SkyStonePosition.LEFT;
                            break;
                        }
                    }
                }
            }
            logger.addDataUpdate("Skystone Position", skyStonePosition);
        } catch (NullPointerException nullPointer) {
            skyStonePosition = team == team.RED ? SkyStonePosition.RIGHT : SkyStonePosition.LEFT;
            logger.addDataUpdate("Error", "No stones detected. Selecting default positions. - " + skyStonePosition);
        }
    }

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

            drive.move(0, moveParams);

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

    protected double calibratedCurrentPosition() {
        double xDistance = globalPositionUpdate.returnXCoordinate() - xOffset;
        double yDistance = globalPositionUpdate.returnYCoordinate() - yOffset;

        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
    }

    protected void resetCalibratedPosition() {
        xOffset = globalPositionUpdate.returnXCoordinate();
        yOffset = globalPositionUpdate.returnYCoordinate();
    }

}