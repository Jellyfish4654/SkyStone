package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.subsystems.imu.IMU;
import org.firstinspires.ftc.teamcode.framework.Utility;
import org.firstinspires.ftc.teamcode.logging.DoubleLogger;
import org.firstinspires.ftc.teamcode.framework.enums.DebugMode;

import java.util.List;

public class Debug{
    
    private List<DcMotor> motors;
    private List<DcMotor> encoders;
    private IMU imu;
    private DebugMode debugMode;
    private DoubleLogger logger;

    public Debug(List<DcMotor> motors, IMU imu, DoubleLogger logger, List<DcMotor> encoders, DebugMode debugMode){

    }

    public void reset(){

    }

    public void report(){

    }

    public void log(){
        
    }
}