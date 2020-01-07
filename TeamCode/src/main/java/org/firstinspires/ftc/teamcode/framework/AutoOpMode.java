package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;
import org.firstinspires.ftc.teamcode.framework.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;

import org.firstinspires.ftc.teamcode.framework.enums.SkyStonePosition;
import org.firstinspires.ftc.teamcode.framework.enums.Team;
import org.firstinspires.ftc.teamcode.framework.enums.Side;

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

    protected DriveTrain.DefaultParams defaultParams;
    protected DriveTrain drive;
    protected DriveTrain.MoveParams params;

    protected final double defaultRampDownPos = 10 * countsPerInch;
    protected final double defaultRampDownEndPos = 5 * countsPerInch;

    protected final double defaultMaxPower = 1;
    protected final double defaultMinPower = .4;

    protected final double defaultRampUpModifier = .15;
    protected final double defaultRampDownModifier = .85;
    protected final double defaultRampDownEndModifier = .95;

    protected final double[] defaultPIDGain = { .03, .03, .03 };

    protected final double defaultCorrectionTime = 500;
    protected final double defaultErrorDistance = countsPerInch * .03;

    public void initMecanum() {
        logger.addDataUpdate("Status", "Initializing Mecanum Drivetrain");
        drive = new Mecanum(Arrays.asList(motors), imu, logger, Arrays.asList(encoders));
        defaultParams = new DriveTrain.DefaultParams(defaultMaxPower, defaultMinPower, defaultRampUpModifier,
                defaultRampDownModifier, defaultRampDownEndModifier, defaultPIDGain, defaultCorrectionTime,
                defaultErrorDistance);
    }

    public void initVision(double confidence) {
        stoneDetector.initVuforia(this, webCam);
        stoneDetector.initTfod(confidence); // 0.55?
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
                        case 2:
                            skyStonePosition = SkyStonePosition.LEFT;
                        default:
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
                        case 2:
                            skyStonePosition = SkyStonePosition.RIGHT;
                        default:
                            break;
                        }
                    }
                }
            }
            logger.addDataUpdate("Skystone Position", skyStonePosition);
        } catch (NullPointerException nullPointer) {
            skyStonePosition = team == team.RED ? SkyStonePosition.LEFT : SkyStonePosition.RIGHT;
            logger.addDataUpdate("Error", "No skystone detected. Selecting default positions. - " + skyStonePosition);
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
            params = new DriveTrain.MoveParams(distance, moveAngle, targetOrientation, defaultParams);
            params.maxPower = power;
            params.minPower = power;
            drive.move(0, params);
            telemetry.addData("Encoder Distance", distance / countsPerInch);
            telemetry.addData("X Distance", xDistance / countsPerInch);
            telemetry.addData("Y Distance", yDistance / countsPerInch);
            telemetry.addData("Move Angle", moveAngle);
            telemetry.update();
            return true;
        } else {
            return false;
        }
    }
}