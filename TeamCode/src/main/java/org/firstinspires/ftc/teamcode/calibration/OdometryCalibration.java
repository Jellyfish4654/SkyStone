package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.framework.enums.Motors;
import org.firstinspires.ftc.teamcode.framework.BaseOpMode;

import java.io.File;

 //RUNS CLOCKWISE
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends BaseOpMode {

    final double PIVOT_SPEED = 0.5;

    // The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE
    // FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    double horizontalTickOffset = 0;

    // Text files to write the values to. The files are stored in the robot
    // controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardware();
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(imu.getZAngle() < 90 && opModeIsActive()){
            motors[Motors.FR].setPower(-PIVOT_SPEED);
            motors[Motors.BR].setPower(-PIVOT_SPEED);
            motors[Motors.FL].setPower(PIVOT_SPEED);
            motors[Motors.BL].setPower(PIVOT_SPEED);
            if(imu.getZAngle() < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = imu.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(encoders[Motors.E_VL].getCurrentPosition()) + (Math.abs(encoders[Motors.E_VR].getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI* countsPerInch);

        horizontalTickOffset = encoders[Motors.E_H].getCurrentPosition()/Math.toRadians(imu.getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", imu.getZAngle());
            telemetry.addData("Vertical Left Position", -encoders[Motors.E_VL].getCurrentPosition());
            telemetry.addData("Vertical Right Position", encoders[Motors.E_VR].getCurrentPosition());
            telemetry.addData("Horizontal Position", encoders[Motors.E_H].getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    private void setPowerAll(double fr, double br, double fl, double bl) {
        motors[Motors.FR].setPower(fr);
        motors[Motors.BR].setPower(br);
        motors[Motors.FL].setPower(fl);
        motors[Motors.BL].setPower(bl);
    }
}
