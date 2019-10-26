package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.framework.drivetrain.IDriveTrain;
import org.firstinspires.ftc.teamcode.framework.drivetrain.Mecanum;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;

import java.util.ArrayList;

@Autonomous(name = "AutoMec Test")
public class AutoMec extends LinearOpMode {
    Telemetry telemetry;
    IDriveTrain drive;

    DcMotor mFrontLeft, mFrontRight, mBackLeft, mBackRight; // Declare mecanum motors
    DcMotor eVerticalLeft, eVerticalRight, eHorizontal, eHorizontalEmpty; // Declares odometry encoder (last one is
                                                                          // empty)
    ArrayList motors, encoders;

    IIMU imu;
    BNO055IMU boschIMU;

    final double defaultMaxPower = .9;
    final double defaultMinPower = .3;
    final double defaultMinPowerPivot = .15;

    final double defaultRampUpModifier = .2;
    final double defaultRampDownModifer;
    final double defaultRampEndModifer;
    final double defaultErrorDistance = 10;
    final double defaultCorrectionTime = 500; // ms

    final double[] defaultPIDGain = { .02 };
    final double countsPerMM;

    @Override
    public void runOpMode() throws IntterruptedException {

        // Init Drivetrain
        drive = new MecanumDrive(motors, imu, telemetry, encoders);
    }

    public void declareStonePositions() {

    }

    public void move(double targetPosition, double moveAngle, double endOrientationAngle) {
        double rampDownTargetPosition = targetPosition * .8;
        double rampUpTargetPosition = targetPosition * .1;
        double rampDownEnd = targetPosition * .9;

        drive.softEncoderReset();
        while (opModeIsActive() && drive.move(drive.getEncoderDistance(), targetPosition, rampDownTargetPosition,
                rampUpTargetPosition, rampDownEnd, defaultMaxPower, defaultMinPower, moveAngle, defaultPIDGain,
                endOrientationAngle, defaultErrorDistance, defaultCorrectionTime))
            ;
        drive.stop();

    }
}