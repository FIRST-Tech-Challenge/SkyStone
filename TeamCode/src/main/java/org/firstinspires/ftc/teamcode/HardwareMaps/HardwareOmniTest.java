package org.firstinspires.ftc.teamcode.HardwareMaps;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareOmniTest {
    //declare all 4 motors as DcMotor to be used furthermore
    public DcMotor motor_front_right = null;
    public DcMotor motor_front_left = null;
    public DcMotor motor_back_right = null;
    public DcMotor motor_back_left = null;

    // State used for updating telemetry
    private HardwareMap hwMxap           =  null;

    /* Constructor */
    public HardwareOmniTest(HardwareMap hwMap){
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        //super(hwMap);
        //initialize motors with directs to Expansion Hub
        motor_front_right = hwMap.get(DcMotor.class, "motor0");
        motor_front_left = hwMap.get(DcMotor.class, "motor1");
        motor_back_right = hwMap.get(DcMotor.class, "motor2");
        motor_back_left = hwMap.get(DcMotor.class, "motor3");

        //set all motors to 0 to stop possible errors caused by not doing this.
        motor_front_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_right.setPower(0);
        motor_back_left.setPower(0);

        //set all motors to run without Encoder because we had a lot of issues with them e.g. stuttering MAY BE REMOVED SOON SINCE I WILL TEST THEM ONCE AGAIN
        motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all motors to brake if they`re set to no power especially because we got a extending arm which needs do stay in place MAY BE REMOVED IF UNKNOWN PROBLEMS POP UP
        motor_front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
