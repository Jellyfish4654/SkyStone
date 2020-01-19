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

    State state = State.MECANUM;

    double xLeft1, yLeft1, pivot;
    double joystickAngle, joystickMagnitude;
    double motionAngle, motionMagnitude, motionX, motionY;
    double robotAngle;
    double speedMultiplier;

    double[] drivePower = new double[4];

    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Status", "Loading JellyTele");
        initHardware();

        logger.addDataUpdate("Status", "Initialization Complete");
        while (!opModeIsActive()) {
            telemetry.addData("Status", "Initialization Complete");

            if (gamepad1.dpad_up) {
                debugMode = DebugMode.NONE;
            } else if (gamepad1.dpad_left) {
                debugMode = DebugMode.PARTIAL;
            } else if (gamepad1.dpad_right) {
                debugMode = DebugMode.ALL;
            }

            telemetry.addData("Debug Mode", debugMode);
            telemetry.update();
        }
        // START OP MODE
        timer.reset();
        initGlobalPosition();

        logger.addData("Status", "JellyTele Active");

        while (opModeIsActive()) {
            telemetry.addData("JellyTele Drive State", state);

            if (gamepad1.dpad_left) {
                fieldCentric = true;
            } else if (gamepad1.dpad_right) {
                fieldCentric = false;
            } else if (gamepad1.dpad_down && gamepad1.a) {
                fieldCentric = true;
                imu.setAsZero();
            }

            xLeft1 = gamepad1.left_stick_x;
            yLeft1 = gamepad1.left_stick_y;
            pivot = gamepad1.right_stick_x;

            speedMultiplier = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);

            if (pivot < 0 && pivot > -0.1)
                pivot = -0.25;
            else if (pivot > 0 && pivot < 0.1)
                pivot = 0.25;

            if (fieldCentric)
                robotAngle = imu.getZAngle();
            else
                robotAngle = 0;

            joystickAngle = Math.toDegrees(Math.atan2(xLeft1, yLeft1));
            joystickMagnitude = Math.sqrt(Math.pow(xLeft1, 2) + Math.pow(yLeft1, 2));

            motionAngle = joystickAngle - robotAngle;

            if (joystickMagnitude > 1) {
                motionMagnitude = 1;
            } else {
                motionMagnitude = joystickMagnitude;
            }

            motionX = motionMagnitude * (Math.sin(Math.toRadians(motionAngle)));
            motionY = motionMagnitude * (Math.cos(Math.toRadians(motionAngle)));

            drivePower[0] = motionY * Math.abs(motionY) - motionX * Math.abs(motionX) - pivot;
            drivePower[1] = motionY * Math.abs(motionY) + motionX * Math.abs(motionX) - pivot;
            drivePower[2] = motionY * Math.abs(motionY) + motionX * Math.abs(motionX) + pivot;
            drivePower[3] = motionY * Math.abs(motionY) - motionX * Math.abs(motionX) + pivot;

            telemetry.addData("Time Left in Match", 120 - timer.seconds());
        }

        for (int i = 0; i < drivePower.length; i++) {
            if (drivePower[i] > 1) {
                drivePower[i] = 1;
            } else if (drivePower[i] < -1) {
                drivePower[i] = -1;
            }
        }

        for (int i = 0; i < drivePower.length; i++) {
            drivePower[i] = drivePower[i] * speedMultiplier;
        }

    }
}
