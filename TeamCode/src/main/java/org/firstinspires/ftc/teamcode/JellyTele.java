package org.firstinspires.ftc.teamcode;

import java.util.Locale;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.BNO055;

import org.firstinspires.ftc.teamcode.framework.enums.Motor;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;

@TeleOp(name = "SkyStone JellyTele", group = "Iterative Opmode")
public class JellyTele extends BaseOpMode {
    private static enum State {
        DRIVE, MECANUM, TANK
    }

    boolean fieldCentric = true;

    @Override
    public void runOpMode() throws InterruptedException {
        logger.addDataUpdate("Op Status", "Loading JellyTele");
        initHardware();

        logger.addDataUpdate("Status", "Initializing IMU (Part 2)");
        imu.initialize();

        waitForStart();

        State state = State.DRIVE;

        while (!opModeIsActive()) {
            if (gamepad1.dpad_up) {
                debugMode = DebugMode.NONE;
            }
            if (gamepad1.dpad_right) {
                debugMode = DebugMode.MECANUM;
            }
            if (gamepad1.dpad_down) {
                debugMode = DebugMode.TANK;
            }
        }

        /*
         ** Dpad** Up - DRIVE Down - TANK Left - Mecanum2 Right Mecanum
         ** 
         * Intake** Left Trigger (G2) Reverse with A + Left Trigger (G2)
         * 
         */

        // START
        logger.addData("Op Status", "Running JellyTele");
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

            double mult = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);
            double x = gamepad1.left_stick_x, y = gamepad1.left_stick_y;
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

            intake(gamepad2.a ? -gamepad2.left_trigger : gamepad2.left_trigger);

        }
        logger.update();
    }

    private void setPowers(double mult, double frontRight, double backRight, double frontLeft, double backLeft) {
        motors[Motor.FR].setPower(frontRight * mult);
        motors[Motor.BR].setPower(backRight * mult);
        motors[Motor.FL].setPower(frontLeft * mult);
        motors[Motor.BL].setPower(backLeft * mult);
        logger.addData("Power State", mult);
    }

    protected void setMecanumPowers(double mult, double angle, double power) {
        double sin = Math.sin(angle - Math.PI / 4);
        double cos = Math.cos(angle - Math.PI / 4);

        setPowers(mult, power * cos, power * sin, power * sin, power * cos);
    }
}