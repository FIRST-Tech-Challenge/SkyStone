/* This abstract class is used to create an abstract for all Chassis since this season there will be more than one.
*  Here all general settings for the (yet only) motors which are used by all other inherited basis classes are set.
*  created by coolPseudonym & dreadjack
 */

package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class HardwareChassis {
    //declare all 4 motors as DcMotor to be used furthermore
    public DcMotor motor_front_right = null;
    public DcMotor motor_front_left = null;
    public DcMotor motor_back_right = null;
    public DcMotor motor_back_left = null;

    public Servo servo_port0 = null;
    public Servo servo_port1 = null;
    public Servo servo_port2 = null;
    public Servo servo_port3 = null;
    public Servo servo_port4 = null;
    public Servo servo_port5 = null;

    //declare a variable to get easier use of the right Hardwaremap
    private HardwareMap hwmap = null;

    /**
     * Initializes the class with a correct hardware map
     * @param ahwMap hardware map object
     */
    public HardwareChassis(HardwareMap ahwMap) {
        //run init hands over hardwaremap of right layout
        init(ahwMap);
        setDirections();
    }

    /**
     * Should be run instantly when the hardware map is received,
     * eg when the constructor is accessed.
     * Inits all motors and sets them.
     * @param hwMap The hardware map from the calling op mode
     */
    public void init(HardwareMap hwMap) {
        //initialize motors with directs to Expansion Hub
        motor_front_right = hwMap.get(DcMotor.class, "motor_front_right");
        motor_front_left = hwMap.get(DcMotor.class, "motor_front_left");
        motor_back_right = hwMap.get(DcMotor.class, "motor_back_right");
        motor_back_left = hwMap.get(DcMotor.class, "motor_back_left");

        //set all motors to 0 to stop possible errors caused by not doing this.
        motor_front_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_right.setPower(0);
        motor_back_left.setPower(0);

        //set all motors to run without Encoder because we had a lot of issues with them e.g. stuttering MAY BE REMOVED SOON SINCE I WILL TEST THEM ONCE AGAIN
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set all motors to brake if they`re set to no power especially because we got a extending arm which needs do stay in place MAY BE REMOVED IF UNKNOWN PROBLEMS POP UP
        motor_front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //initialize servos
        servo_port0 = hwMap.get(Servo.class, "servo_port0");
        servo_port1 = hwMap.get(Servo.class, "servo_port1");
        servo_port2 = hwMap.get(Servo.class, "servo_port2");
        servo_port3 = hwMap.get(Servo.class, "servo_port3");
        servo_port4 = hwMap.get(Servo.class, "servo_port4");
        servo_port5 = hwMap.get(Servo.class, "servo_port5");
    }

    //force a method to set the right directions per wheel-layout
    abstract protected void setDirections();
}