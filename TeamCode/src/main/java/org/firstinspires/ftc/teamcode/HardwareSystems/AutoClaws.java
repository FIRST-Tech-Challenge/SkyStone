package org.firstinspires.ftc.teamcode.HardwareSystems;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Timer;

public class AutoClaws {

    private String side;
    private Timer timer;
    private RobotHardware hardware;

    /* Auto claw positions:    Left        Right
     * Initialized Gripper     *0.575      *0.927
     * Initialized Flipper     *0.29       *0.3469
     * Flipper Down            *0.825      *0.879
     * Gripper Clamped         *0.3        *0.91
     * Flipper Holding         *0.402      *0.482
     * Gripper Open            *0.55       *0.575
     * Flipper Prime           *           *0.825
     * Gripper Prime           *           *0.61
     */

    double rightInitializedGripper = 0.927;
    double rightInitializedFlipper = 0.3469;
    double rightFlipperDown = 0.879;
    double rightGripperClamped = 0.91;
    double rightFlipperHolding = 0.482;
    double rightGripperOpen = 0.575;


    public AutoClaws(RobotHardware hardware, String side, Timer timer){
        this.side = side;
        this.timer = timer;
        this.hardware = hardware;
    }

    public void initialize(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.3469);
            hardware.autoGrabberRight.setPosition(0.927);
        }else if(side.equals("BLUE")){
            hardware.autoFlipperLeft.setPosition(0.29);
            hardware.autoGrabberLeft.setPosition(0.2);
        }
    }

    public void prime(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.79);
            hardware.autoGrabberRight.setPosition(0.6);
        }else if(side.equals("BLUE")){
            hardware.autoFlipperLeft.setPosition(0.79);
            hardware.autoGrabberLeft.setPosition(0.68);
        }
    }

    public void firstPrime(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.79);
            hardware.autoGrabberRight.setPosition(0.6);
        }else if(side.equals("BLUE")){
            hardware.autoFlipperLeft.setPosition(0.59);
            hardware.autoGrabberLeft.setPosition(0.68);
        }
    }

    public void grabBlock(){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(0.894);
            timer.waitMillis(260);
            hardware.autoGrabberRight.setPosition(0.872);
        }else if(side.equals("BLUE")){
            hardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(260);
            hardware.autoGrabberLeft.setPosition(0.3);
            timer.waitMillis(200);
        }
    }

    public void storeBlock(){
        if(side.equals("RED")){
            timer.waitMillis(325);
            hardware.autoFlipperRight.setPosition(0.445);
            hardware.autoGrabberRight.setPosition(0.972);
            timer.waitMillis(200);
        }else if(side.equals("BLUE")){
            timer.waitMillis(325);
            hardware.autoFlipperLeft.setPosition(0.402);
            hardware.autoGrabberLeft.setPosition(0.3);
            timer.waitMillis(150);
        }
    }
    public void depositBlock(){
        if(side.equals("RED")){
            hardware.autoGrabberRight.setPosition(0.575);
            hardware.autoFlipperRight.setPosition(0.6);
            timer.waitMillis(250);
            hardware.autoFlipperRight.setPosition(0.177);
        }else if(side.equals("BLUE")){
            timer.waitMillis(100);

            hardware.autoGrabberLeft.setPosition(0.55);
            hardware.autoFlipperLeft.setPosition(0.825);
            timer.waitMillis(285);
            hardware.autoFlipperLeft.setPosition(0.29);
            hardware.autoGrabberLeft.setPosition(0.4);
        }
    }

    public void depositBlockThrow(){
        if(side.equals("RED")){
            timer.waitMillis(40);
            hardware.autoFlipperRight.setPosition(5);//
            hardware.autoGrabberRight.setPosition(0.55);//open
            timer.waitMillis(30);
            hardware.autoFlipperRight.setPosition(0.482);
            timer.waitMillis(10);
        }else if(side.equals("BLUE")){
            timer.waitMillis(40);
            hardware.autoGrabberLeft.setPosition(0.55);
            hardware.autoFlipperLeft.setPosition(0.6);
            timer.waitMillis(85);
            hardware.autoFlipperLeft.setPosition(0.29);
            hardware.autoGrabberLeft.setPosition(0.4);
            timer.waitMillis(10);

        }
    }

}

/*
    public void setPositions(double gripperPosition, double flipperPosition){
        if(side.equals("RED")){
            hardware.autoFlipperRight.setPosition(flipperPosition);
            hardware.autoGrabberRight.setPosition(gripperPosition);
        }
    }
 */