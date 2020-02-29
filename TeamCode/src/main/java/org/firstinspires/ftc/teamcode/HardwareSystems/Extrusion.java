package org.firstinspires.ftc.teamcode.HardwareSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Controllers.Pid;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

/* An extrusion class to automate all extrusion tasks
The direction of the motors must be such that a positive power causes the encoder to increase
*/

public class Extrusion {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    private int liftMax = 99999;
    private int liftCurrent = 0;
    private int liftTarget = 0;
    private int[] levels = {0, 185, 420, 655, 871, 1067, 1261, 1465, 1690, 1900, 2110};

    private double weight = 0.2; //F term of the virtual PIDF controller for this lift
    private Pid controller;

    public Extrusion(LinearOpMode opMode, RobotHardware hardware){
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void initialize(){
        //Set Run Mode
        hardware.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Reversing the motors so they don't fight and move in the right direction
        hardware.liftRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set them to brake
        hardware.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Reset the encoders
        hardware.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = new Pid(0.06, 0.001, 0.01, 50, 0.5, 0);

    }

    public void setTargetPosition(int position){
        if(position > 0 && position < liftMax){
            liftTarget = position;
        }
    }

    public void goToLevelLinear(int blockLevel){ //1 means the "first block level"
        setTargetPosition(levels[blockLevel-1]);

        while(opMode.opModeIsActive()){
            update();
        }

    }

    public void update(){
        liftCurrent = (hardware.liftLeft.getCurrentPosition() + hardware.liftRight.getCurrentPosition())/2;
        controller.update(liftTarget, liftCurrent);
        setPower(controller.correction + weight);
    }

    public void setPower(double power){
        hardware.liftRight.setPower(power);
        hardware.liftLeft.setPower(power);
    }

}