package org.firstinspires.ftc.teamcode.HardwareMaps;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareChassis {
    //declare all motors as DcMotor to be used furthermore
    public DcMotor motor_front_right = null;
    public DcMotor motor_front_left = null;
    public DcMotor motor_rear_right = null;
    public DcMotor motor_rear_left = null;
    public DcMotor motor_lift_left = null;
    public DcMotor motor_lift_right = null;
    public DcMotor motor_extender = null;

    // declare sensors
    public ColorSensor color_front = null;
    public ColorSensor color_back = null;
    public DigitalChannel touch_left = null;
    public DigitalChannel touch_right = null;

    public Servo servo_grab;
    public Servo servo_claw_right;
    public Servo servo_claw_left;
    public Servo servo_capstone;

    // State used for updating telemetry
    private HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareChassis(HardwareMap hwMap){
        //super(hwMap);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        //// MOTORS

        //initialize motors with directs to Expansion Hub
        this.motor_front_left =  hwMap.get(DcMotor.class, "hub1_motorport2");
        this.motor_rear_right =  hwMap.get(DcMotor.class, "hub1_motorport1");
        this.motor_front_right = hwMap.get(DcMotor.class, "hub1_motorport0");
        this.motor_rear_left =   hwMap.get(DcMotor.class, "hub1_motorport3");

        this.motor_lift_left =   hwMap.get(DcMotor.class, "hub2_motorport2");
        this.motor_lift_right =  hwMap.get(DcMotor.class, "hub2_motorport1");

        this.motor_extender =       hwMap.get(DcMotor.class, "hub2_motorport3");

        //set all motors to 0 to stop possible errors caused by not doing this.
        this.motor_front_right.setPower(0);
        this.motor_front_left.setPower(0);
        this.motor_rear_right.setPower(0);
        this.motor_rear_left.setPower(0);
        this.motor_lift_left.setPower(0);
        this.motor_lift_right.setPower(0);
        this.motor_extender.setPower(0);

        //set all motors to run with Encoders
        this.motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_lift_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor_extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all motors to brake if they`re set to no power especially because we got a extending arm which needs do stay in place MAY BE REMOVED IF UNKNOWN PROBLEMS POP UP
        this.motor_front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //// SENSORS
        this.color_front =    hwMap.get(ColorSensor.class, "hub1_colorport1");
        this.color_back =   hwMap.get(ColorSensor.class, "hub2_colorport1");

        this.touch_left =    hwMap.get(DigitalChannel.class, "hub2_digitalport1");
        this.touch_right =   hwMap.get(DigitalChannel.class, "hub2_digitalport3");

        this.servo_grab = hwMap.get(Servo.class, "hub1_servoport1");
        this.servo_claw_left = hwMap.get(Servo.class, "hub1_servoport5");
        this.servo_claw_right = hwMap.get(Servo.class, "hub1_servoport4");
        this.servo_capstone = hwMap.get(Servo.class, "hub2_servoport0");
    }
}