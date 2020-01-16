package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;

import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;

@Disabled
@TeleOp(name = "SkyStone JellyTele2", group = "Tele V2")

public class JellyTele2 extends BaseOpMode {
    private static enum State {
        DRIVE, MECANUM, TANK
    }

    private boolean fieldCentric = false;

    State state = MECANUM;

    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Status", "Loading JellyTele");
        initHardware();

        logger.addDataUpdate("Status", "Initialization Complete");
        while (!opModeIsActive()) {
            telemetry.addData("Status", "Initialization Complete");

            if (gamepad2.dpad_up) {
                debugMode = DebugMode.NONE;
            } else if (gamepad2.dpad_left) {
                debugMode = DebugMode.PARTIAL;
            } else if (gamepad2.dpad_right) {
                debugMode = DebugMode.ALL;
            }

            telemetry.addData("Debug Mode", debugMode);
            telemetry.update();
        }

        initGlobalPosition();

        logger.addData("Status", "JellyTele Active");

        while (opModeIsActive()) {

        }

    }
}
