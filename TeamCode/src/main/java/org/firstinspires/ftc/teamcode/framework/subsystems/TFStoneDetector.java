package org.firstinspires.ftc.teamcode.framework.subsystems;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TFStoneDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AazZIlb/////AAABmRbCE9nGwUxMsIXmlS2x1+NMRNQ8Hz20HMiHWJeSBk9fXUYA5XnNqK6z4fAkSQmPHAxHfdp6DuLU6Qq1dVRe+sGvRuRPO15KyqgDIMqRAtlQQLOjyo0wuJF73BrtYGSWI9/axd7kUXLRBR9gurnTRqVxVLp8ktFsH05GoL4AR8fNP/UNJiEs/v7QQ5aBtYs4qhOGspKEV0YI/s+2ljKdWJpHcLRpu9jJYoFrbp47FZiRyK0L4VRWQ5dfxOUKyiCmQgID3j4ZHj0PGvwzz/c4n6OZxz7SrXW8pLPkfZE4H1+g6/bypvqRv8WZxrNgduI9IGGvIC5A+5IRqVcmqkTNIkIAgbAjV7mg/AeWx329RwF6";

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    WebcamName webcamName;

    private LinearOpMode linearOpMode;

    public void initVuforia(LinearOpMode opmode) {
        linearOpMode = opmode;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        
        // Phone cam and webcam
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = linearOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
      
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod(Double minConfidence) {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = linearOpMode.hardwareMap.appContext.getResources()
                    .getIdentifier("tfodMonitorViewId", "id", linearOpMode.hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = minConfidence;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            linearOpMode.telemetry.addData(">>", "TensorFlow Init");
            linearOpMode.telemetry.update();
        } else {
            linearOpMode.telemetry.addData(">>", "Check Phone. TFOD Incompatibility Error");
            linearOpMode.telemetry.update();
        }
    }

    public void activateTF() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void shutdownTF() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void detectStone() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            linearOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                linearOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                linearOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                linearOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
            linearOpMode.telemetry.update();
        }
    }
}