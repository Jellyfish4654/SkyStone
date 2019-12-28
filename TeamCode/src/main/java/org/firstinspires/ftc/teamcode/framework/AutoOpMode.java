package org.firstinspires.ftc.teamcode.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.vision.TFStoneDetector;

import org.firstinspires.ftc.teamcode.framework.enums.SkyStonePosition;
import org.firstinspires.ftc.teamcode.framework.enums.Team;
import org.firstinspires.ftc.teamcode.framework.enums.Side;

import java.util.List;

public abstract class AutoOpMode extends BaseOpMode {

    protected List<Recognition> updatedRecognitions;

    protected Team team = Team.RED;
    protected Side side = Side.STONE;
    protected SkyStonePosition skyStonePosition = null;
    protected TFStoneDetector stoneDetector = new TFStoneDetector();

    protected static final String webCam = "Webcam 1";

    public void initVision(double confidence) {
        stoneDetector.initVuforia(this, webCam);
        stoneDetector.initTfod(confidence); // 0.55?
    }

    public void getStonePositions() {
        updatedRecognitions = stoneDetector.detectStone();

        // sort stones, right to left
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
    }
}