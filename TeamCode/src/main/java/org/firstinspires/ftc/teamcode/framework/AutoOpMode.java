import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

public abstract class AutoOpMode extends BaseOpMode {
    protected static enum Team {
        RED, BLUE
    }

    protected Team team;

    protected StonePosition getStonePosition() {
        logger.addData("stone status", "Declaring Stone Positions");
        stoneDetector.activateTF();
        logger.addData("stone event", "TensorFlow Activated");
        logger.update();

        List<Recognition> updatedRecognitions = stoneDetector.detectStone();
        boolean validDetectionStatus = false;
        timer.reset();

        while (!validDetectionStatus && timer.milliseconds() < 500 && !isStopRequested()) {
            updatedRecognitions = stoneDetector.detectStone();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2)
                    validDetectionStatus = true;
                else
                    validDetectionStatus = false;
            } else
                validDetectionStatus = false;
            if (timer.seconds() > 5)
                break;
        }

        StonePosition stonePosition = null;

        if (validDetectionStatus){
            logger.addData("stone event", "Valid Detection Confirmed");
            logger.update();
            for(int i = 0; updatedRecognitions.size()>i; i++){
                if(updatedRecognitions.get(i).getLabel()=="Skystone"){
                    logger.addData("stone status", "right edge %f", updatedRecognitions.get(i).getRight());
                    if (updatedRecognitions.get(i).getRight()>=500){ // change this value
                        stonePosition = StonePosition.RIGHT;
                        logger.addData("stone status", "Stone = RIGHT");
                    } else {
                        stonePosition = StonePosition.CENTER;
                        logger.addData("stone status", "Stone = CENTER");
                    }
                }
            }
            if (stonePosition == null) {
                stonePosition = StonePosition.LEFT;
                logger.addData("stone status", "Stone = LEFT");
            }
        } else {
            logger.addData("stone status", "Invalid stone detection");
            logger.update();
        }

        return stonePosition;

        // Overview
        // Reset origin, Check position one, if can be determined return
        // DO NOT RESET ORIGIN Check position two, if can be determined return
        // DO NOT RESET ORIGIN Check position three, if can be determined return
        // DO NOT RESET ORIGIN Move to last pos and return
    }

    protected void getSkyStone() {
        StonePosition stonePosition = getStonePosition();
        while (opModeIsRunning()) {
            if (stonePosition == )
            idle();
        }
    }
}