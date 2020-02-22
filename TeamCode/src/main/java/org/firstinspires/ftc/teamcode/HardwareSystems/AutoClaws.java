package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

public class AutoClaws {

    private String side;
    private Timer timer;

    /* Auto claw positions:    Left        Right
     * Initialized Gripper     *0.275      *1.0
     * Initialized Flipper     *0          *0.251
     * Flipper Down            *0.825      *0.99
     * Gripper Clamped         *0.3        *0.985
     * Flipper Holding         *0.402      *0.482
     * Gripper Open            *0.55       *0.575
     */

    public AutoClaws(String side, Timer timer){
        this.side = side;
        this.timer = timer;
    }

    public void initialize(){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(0.29);
            RobotHardware.autoGrabberRight.setPosition(1);
        }else if(side.equals("BLUE")){
            RobotHardware.autoFlipperLeft.setPosition(0);
            RobotHardware.autoGrabberLeft.setPosition(0.275);
        }
    }

    public void prime(){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(0.7);
            RobotHardware.autoGrabberRight.setPosition(0.575);
        }else{
            RobotHardware.autoFlipperLeft.setPosition(0.79);
            RobotHardware.autoGrabberLeft.setPosition(0.55);
        }
    }

    public void grabBlock(){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(0.99);
            timer.waitMillis(200);
            RobotHardware.autoGrabberRight.setPosition(0.985);
        }else{
            RobotHardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(200);
            RobotHardware.autoGrabberLeft.setPosition(0.3);
        }
    }

    public void storeBlock(){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(0.482);
            RobotHardware.autoGrabberRight.setPosition(0.985);
        }else{
            RobotHardware.autoFlipperLeft.setPosition(0.402);
            RobotHardware.autoGrabberLeft.setPosition(0.3);
        }
    }

    public void depositBlock(){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(0.6);
            timer.waitMillis(150);
            RobotHardware.autoGrabberRight.setPosition(0.575);
            timer.waitMillis(150);
            RobotHardware.autoFlipperRight.setPosition(0.482);
        }else{
            RobotHardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(150);
            RobotHardware.autoGrabberLeft.setPosition(0.55);
            timer.waitMillis(150);
            RobotHardware.autoFlipperLeft.setPosition(0.402);
        }
    }

    public void setPositions(double gripperPosition, double flipperPosition){
        if(side.equals("RED")){
            RobotHardware.autoFlipperRight.setPosition(flipperPosition);
            RobotHardware.autoGrabberRight.setPosition(gripperPosition);
        }
    }


}
