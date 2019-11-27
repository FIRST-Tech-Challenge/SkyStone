package org.firstinspires.ftc.teamcode.HardwareMaps;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareClampTest {
    //declare all 4 motors as DcMotor to be used furthermore
    public DcMotor motor_clamp_extender_left = null;
    public DcMotor motor_clamp_extender_right = null;
    public DigitalChannel button = null;

    // State used for updating telemetry
    private HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareClampTest(HardwareMap hwMap){
        //super(hwMap);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        //------MOTORS-------
        //initialize motors with directs to Expansion Hub
        motor_clamp_extender_left = hwMap.get(DcMotor.class, "motor0");
        motor_clamp_extender_right = hwMap.get(DcMotor.class, "motor1");

        //set all motors to 0 to stop possible errors caused by not doing this.
        motor_clamp_extender_left.setPower(0);
        motor_clamp_extender_right.setPower(0);

        //set all motors to run with Encoders
        motor_clamp_extender_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_clamp_extender_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all motors to brake if they`re set to no power especially because we got a extending arm which needs do stay in place MAY BE REMOVED IF UNKNOWN PROBLEMS POP UP
        motor_clamp_extender_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_clamp_extender_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //-------SENSORS--------
        button = hwMap.digitalChannel.get("button");
    }
}
