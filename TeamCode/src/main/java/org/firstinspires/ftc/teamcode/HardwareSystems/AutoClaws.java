package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

public class AutoClaws {

    private String side;
    private Timer timer;
    private RobotHardware hardware;

    /* Auto claw positions:    Left        Right
     * Initialized Gripper     *0.575      *1.0
     * Initialized Flipper     *0.29       *0.251
     * Flipper Down            *0.825      *0.99
     * Gripper Clamped         *0.3        *0.985
     * Flipper Holding         *0.402      *0.482
     * Gripper Open            *0.55       *0.575
     */

    public AutoClaws(RobotHardware hardware, String side, Timer timer){
        this.side = side;
        this.timer = timer;
        this.hardware = hardware;
    }

    public void initialize(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.251);
            hardware.autoGrabberRight.setPosition(1);
        }else if(side.equals("BLUE")){
            hardware.autoFlipperLeft.setPosition(0.29);
            hardware.autoGrabberLeft.setPosition(0.2);
        }
    }

    public void prime(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.7);
            hardware.autoGrabberRight.setPosition(0.5);
        }else{
            hardware.autoFlipperLeft.setPosition(0.79);
            hardware.autoGrabberLeft.setPosition(0.68);
        }
    }

    public void firstPrime(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.5);
            hardware.autoGrabberRight.setPosition(0.5);
        }else{
            hardware.autoFlipperLeft.setPosition(0.59);
            hardware.autoGrabberLeft.setPosition(0.68);
        }
    }

    public void grabBlock(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.99);
            timer.waitMillis(260);
            hardware.autoGrabberRight.setPosition(1.0);
        }else{
            hardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(260);
            hardware.autoGrabberLeft.setPosition(0.3);
        }
    }

    public void storeBlock(){
        if(side.equals("RED")){
            timer.waitMillis(300);
            hardware.autoFlipperRight.setPosition(0.482);
            hardware.autoGrabberRight.setPosition(0.985);
            timer.waitMillis(100);
        }else{
            timer.waitMillis(325);
            hardware.autoFlipperLeft.setPosition(0.402);
            hardware.autoGrabberLeft.setPosition(0.3);
            timer.waitMillis(100);
        }
    }
    public void depositBlock(){
        if(side.equals("RED")){
            hardware.autoGrabberRight.setPosition(0.575);
            hardware.autoFlipperRight.setPosition(0.6);
            timer.waitMillis(250);
            hardware.autoFlipperRight.setPosition(0.482);
        }else{
            hardware.autoGrabberLeft.setPosition(0.55);
            hardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(250);
            hardware.autoFlipperLeft.setPosition(0.402);
        }
    }

    public void depositBlockThrow(){
        if(side.equals("RED")){
            hardware.autoGrabberRight.setPosition(0.575);
            timer.waitMillis(70);
            hardware.autoFlipperRight.setPosition(0.6);
            timer.waitMillis(250);
            hardware.autoFlipperRight.setPosition(0.482);
        }else{
            hardware.autoGrabberLeft.setPosition(0.55);
            timer.waitMillis(70);
            hardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(250);
            hardware.autoFlipperLeft.setPosition(0.402);
        }
    }
    public void setPositions(double gripperPosition, double flipperPosition){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(flipperPosition);
            hardware.autoGrabberRight.setPosition(gripperPosition);
        }
    }


}
