package org.firstinspires.ftc.teamcode.DutchFTCCore;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.GuidanceSubSystem;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.IMUSubSystem;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.SubSystem;
import org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems.MovementSubSystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    //instance
    public static Robot instance;

    //current opmode, set by constructor
    public OpMode opMode;

    //drivetrain
    public DcMotor MotorBackLeft;
    public DcMotor MotorFrontLeft;
    public DcMotor MotorFrontRight;
    public DcMotor MotorBackRight;
    public DcMotor MotorMiddle;
    public BNO055IMU imu;

    public Drivetraintypes drivetrains;

    //list of subsystems
    public List<SubSystem> subSystems;


    public Robot (OpMode _opmode) {
        instance = this;
        opMode = _opmode;
        drivetrains = new Drivetraintypes();
        drivetrains.initDrivetrain(this);
        imu = opMode.hardwareMap.get(BNO055IMU.class,"imu");
        subSystems = new ArrayList<>();
    }

    public void Update (){
        for (SubSystem system : subSystems)
        {
          system.Update();
        }
    }

    /**
     * calls the calibration functions on all sensors
     */
    public void Calibrate(){
        if (IMUSubSystem.instance != null){
            IMUSubSystem.instance.Calibrate();
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                           Create start functions for all subsystems here                                             //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void StartIMUSubSystem () {
        IMUSubSystem a = new IMUSubSystem();
        a.Start();
        subSystems.add(a);
    }

    public void StartMovementSubSystem(){
        MovementSubSystem a = new MovementSubSystem();
        a.Start();
        subSystems.add(a);
    }

    public void StartGuidanceSubSystem () {
        GuidanceSubSystem a = new GuidanceSubSystem();
        a.Start();
        subSystems.add(a);
    }
}