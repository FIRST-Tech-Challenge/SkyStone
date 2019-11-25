package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BBHooks {

    private Servo _leftHookMotor;
    private Servo _rightHookMotor;

    private Servo _skyHookMotor;

    private boolean _hookState;

        public void init(HardwareMap hwmap){

            _leftHookMotor = hwmap.get(Servo.class, "left_hook");
            _rightHookMotor = hwmap.get(Servo.class, "right_hook");
            _skyHookMotor = hwmap.get(Servo.class, "sky_hook");
            _hookState = false;
        }

        public void Latched(){

            _leftHookMotor.setPosition(0);
            _rightHookMotor.setPosition(1);
            _hookState = true;

        }

        public void UnLatched() {

            _leftHookMotor.setPosition(1);
            _rightHookMotor.setPosition(0);
            _hookState = false;
        }

        public void ToggleHook(){

            if (_hookState == false){
                Latched();
            }
            else if (_hookState == true){
                UnLatched();
            }

        }

        public void SkyHookOn(){
            _skyHookMotor.setPosition(1);
        }

        public void SkyHookOff(){
            _skyHookMotor.setPosition(0);
        }


}
