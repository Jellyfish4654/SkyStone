package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;

import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;

@TeleOp(name = "SkyStone JellyTele", group = "Iterative Opmode")
public class JellyTele extends BaseOpMode {
    private static enum State {
        DRIVE, MECANUM, TANK, FIELDMECANUM
    }

    double imuOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Status", "Loading JellyTele");
        initHardware();
        // imuOffset = Double.parseDouble(ReadWriteFile.readFile(autoIMUOffset).trim());
        // imu.setOffset(-imuOffset);

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

        waitForStart();
        initGlobalPosition();
        State state = State.MECANUM;

        /*
         ** Dpad** Up - DRIVE Down - TANK Left - Mecanum2 Right Mecanum
         ** 
         * Intake** Left Trigger (G2) Reverse with A + Left Trigger (G2)
         * 
         */

        // START
        logger.addData("Status", "JellyTele Active");
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                state = State.DRIVE;
                logger.addData("Drive State", "DRIVE");
            } else if (gamepad1.dpad_down) {
                state = State.TANK;
                logger.addData("Drive State", "TANK");
            } else if (gamepad1.dpad_left) {
                state = State.MECANUM;
                logger.addData("Drive State", "MECANUM");
            }

            telemetry.addData("Drive State", state);

            if (debugMode != DebugMode.NONE)
                positionTelemetry();
            if (debugMode != DebugMode.NONE && debugMode != DebugMode.PARTIAL && gamepad2.start)
                positionSave();

            double mult = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);
            double x = -gamepad1.left_stick_x, y = -gamepad1.left_stick_y;
            switch (state) {
            case DRIVE:
                setPowers(mult, y + x, y + x, y - x, y - x);
                break;
            case MECANUM:
                double power2 = Math.sqrt(x * x + y * y);
                double angle2 = Math.atan2(y, x);
                double sin2 = Math.sin(angle2 - Math.PI / 4);
                double cos2 = Math.cos(angle2 - Math.PI / 4);

                double turn = -gamepad1.right_stick_x;

                setPowers(mult, power2 * cos2 - turn, power2 * sin2 - turn, power2 * sin2 + turn, power2 * cos2 + turn);
                break;
            case TANK:
                double left = gamepad1.left_stick_y;
                double right = gamepad1.right_stick_y;
                setPowers(mult, right, right, left, left);
                break;
            }

            if (gamepad2.left_bumper) {
                foundationLeft.setPosition(foundationLeftExtend);
                foundationRight.setPosition(foundationRightExtend);
            } else {
                foundationLeft.setPosition(foundationLeftRetract);
                foundationRight.setPosition(foundationRightRetract);
            }
            
            intake(gamepad2.a ? -gamepad2.left_trigger : gamepad2.left_trigger);
           
            if (gamepad2.start && debugMode == DebugMode.ALL) {
                positionSave();
            }

          
            logger.update();
        }
    }

    private void setPowers(double mult, double frontRight, double backRight, double frontLeft, double backLeft) {
        motors[Motor.FR].setPower(frontRight * mult);
        motors[Motor.BR].setPower(backRight * mult);
        motors[Motor.FL].setPower(frontLeft * mult);
        motors[Motor.BL].setPower(backLeft * mult);
    }

    protected void setMecanumPowers(double mult, double angle, double power) {
        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);

        setPowers(mult, power * cos, power * sin, power * sin, power * cos);
    }
}