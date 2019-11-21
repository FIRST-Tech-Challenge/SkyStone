package org.firstinspires.ftc.teamcode.HardwareMaps;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Consumer;

public class HardwareNeutral extends HardwareMap {
    //declare all 8 motors as DcMotor to be used furthermore
    public DcMotor motor_hub1_port0 = null;
    public DcMotor motor_hub1_port1 = null;
    public DcMotor motor_hub1_port2 = null;
    public DcMotor motor_hub1_port3 = null;

    public DcMotor motor_hub2_port0 = null;
    public DcMotor motor_hub2_port1 = null;
    public DcMotor motor_hub2_port2 = null;
    public DcMotor motor_hub2_port3 = null;

    // State used for updating telemetry
    private HardwareMap hwMap =  null;

    /* Constructor */
    public HardwareNeutral(HardwareMap hwMap){
        super(hwMap);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        //initialize motors with directs to Expansion Hub
        motor_hub1_port0 = hwMap.get(DcMotor.class, "motor_hub1_port0");
        motor_hub1_port1 = hwMap.get(DcMotor.class, "motor_hub1_port1");
        motor_hub1_port2 = hwMap.get(DcMotor.class, "motor_hub1_port2");
        motor_hub1_port3 = hwMap.get(DcMotor.class, "motor_hub1_port3");

        motor_hub2_port0 = hwMap.get(DcMotor.class, "motor_hub2_port0");
        motor_hub2_port1 = hwMap.get(DcMotor.class, "motor_hub2_port1");
        motor_hub2_port2 = hwMap.get(DcMotor.class, "motor_hub2_port2");
        motor_hub2_port3 = hwMap.get(DcMotor.class, "motor_hub2_port3");

        //set all motors to 0 to stop possible errors caused by not doing this.
        motor_hub1_port0.setPower(0);
        motor_hub1_port1.setPower(0);
        motor_hub1_port2.setPower(0);
        motor_hub1_port3.setPower(0);

        motor_hub2_port0.setPower(0);
        motor_hub2_port1.setPower(0);
        motor_hub2_port2.setPower(0);
        motor_hub2_port3.setPower(0);

        //set all motors to run with Encoders
        motor_hub1_port0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub1_port1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub1_port2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub1_port3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_hub2_port0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub2_port1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub2_port2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_hub2_port3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all motors to brake if they`re set to no power especially because we got a extending arm which needs do stay in place MAY BE REMOVED IF UNKNOWN PROBLEMS POP UP
        motor_hub1_port0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub1_port1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub1_port2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub1_port3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_hub2_port0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub2_port1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub2_port2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_hub2_port3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
