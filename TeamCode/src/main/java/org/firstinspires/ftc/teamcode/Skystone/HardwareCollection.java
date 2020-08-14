package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Deprecated
public class HardwareCollection {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    // Intake Motors
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    // Outtake Motors
    public DcMotor outtakeSpool;
    public DcMotor outtakeSpool2;

    // Outtake Servos
    public Servo outtakeExtender;
    public Servo backClamp;
    public Servo frontClamp;
    public Servo intakePusher;

    // Foundation Servos
    public Servo leftFoundation;
    public Servo rightFoundation;

    public DistanceSensor intakeStoneDistance;

    public HardwareMap hardwareMap;

    public LynxModule revHub1;
    public LynxModule revHub2;

    public long currTime;


    public HardwareCollection(HardwareMap hwMap){
        this.hardwareMap = hwMap;

        revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshData1();
        revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 3");
        revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        refreshData2();

        //Map drive motors
        fLeft = getDcMotor("fLeft");
        if (fLeft != null) {
            fLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        fRight = getDcMotor("fRight");
        if (fRight != null) {
            fRight.setDirection(DcMotor.Direction.REVERSE);
        }

        bLeft = getDcMotor("bLeft");
        if (bLeft != null) {
            bLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        bRight = getDcMotor("bRight");
        if (bRight != null) {
            bRight.setDirection(DcMotor.Direction.REVERSE);
        }

        intakeLeft = getDcMotor("intakeLeft");
        if (intakeLeft != null) {
            intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        }

        intakeRight = getDcMotor("intakeRight");
        if (intakeRight != null) {
            intakeRight.setDirection(DcMotor.Direction.FORWARD);
        }

        outtakeSpool = getDcMotor("outtakeSpool");
        if (outtakeSpool != null) {
            outtakeSpool.setDirection(DcMotor.Direction.REVERSE);
        }

        outtakeSpool2 = getDcMotor("outtakeSpool2");
        if (outtakeSpool2 != null) {
            outtakeSpool2.setDirection(DcMotor.Direction.REVERSE);
        }

        outtakeExtender = getServo("outtakeExtender");
        backClamp = getServo("backClamp");
        frontClamp = getServo("frontClamp");
        intakePusher = getServo("intakePusher");

        leftFoundation = getServo("leftFoundation");
        rightFoundation = getServo("rightFoundation");

        intakeStoneDistance = hardwareMap.get(DistanceSensor.class, "intakeStoneDistance");
    }

    public void refreshData1(){
        revHub1.getBulkData();
    }

    public void refreshData2(){
        revHub2.getBulkData();
    }

    public void updateTime(){
        currTime = SystemClock.elapsedRealtime();
    }

    private DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);

        } catch (IllegalArgumentException exception) {
            return null;
        }
    }

    private Servo getServo(String name) {
        try {
            return hardwareMap.servo.get(name);

        } catch (IllegalArgumentException exception) {
            return null;
        }
    }

    private Rev2mDistanceSensor getRev2mDistanceSensor(String name) {
        try {
            return hardwareMap.get(Rev2mDistanceSensor.class, name);

        } catch (IllegalArgumentException exception) {
            return null;
        }
    }
}
