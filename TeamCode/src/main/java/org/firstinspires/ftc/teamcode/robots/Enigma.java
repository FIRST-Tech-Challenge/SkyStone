package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

/**
 * Created by Aila on 2017-08-21.
 */

public class Enigma extends Robot {

    //Motors
    public Motor shoot;
    public Motor intake;

    //Servos
    public FXTCRServo load;

    public Enigma() {
        super();
        motorR.setReverse(true);
        motorL.setReverse(false);

        shoot = new Motor("shoot");
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.setMinimumSpeed(0);

        intake = new Motor("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMinimumSpeed(0);

        load = new FXTCRServo("load");
        load.setPower(0);
    }

    public int distanceToTicksR(int distance){
        double motorTicks = motorR.getNumTiksPerRev();
        double mc = 3.14 * (wheelDiameter * 25.4);
        double tickPerMM = motorTicks/mc;
        int ticks = (int) Math.round(tickPerMM * distance);
        return ticks;
    }

    public int degreesToDistance(int degrees){
        int distance = degrees * 4;
        return distance;
    }

    public int distanceToTicksL(int distance){
        double motorTicks = motorL.getNumTiksPerRev();
        double mc = 3.14 * (wheelDiameter * 25.4);
        double tickPerMM = motorTicks/mc;
        int ticks = (int) Math.round(tickPerMM * distance);
        return ticks;
    }

    public void forward(double power){
        motorR.setPower(power);
        motorL.setPower(power);
    }

    public void reverse(double power){
        motorR.setPower(-power);
        motorL.setPower(-power);
    }

    public void stops(){
        motorR.setPower(0);
        motorL.setPower(0);
    }

    public void forwardPosition(double power, int distance){
        int place = motorR.getBaseCurrentPosition();
        int ticks = place + distanceToTicksR(distance);

        while (RC.l.opModeIsActive() && ticks > place) {
            forward(power);
            place = motorR.getBaseCurrentPosition();
        }
        stops();
    }

    public void reversePosition(double power, int distance){
        int place = motorR.getBaseCurrentPosition();
        int ticks = place - distanceToTicksR(distance);
        if (ticks < place){
            while (RC.l.opModeIsActive() && ticks < place) {
                reverse(power);
                place = motorR.getBaseCurrentPosition();
            }
            stop();
        }
        else{
            stop();
        }
    }

    public void leftTurn(double power, int degrees){
        int distance = degreesToDistance(degrees);
        int place = motorR.getBaseCurrentPosition();
        int ticks = place + distanceToTicksR(distance);
        while (RC.l.opModeIsActive() && ticks > place) {
            motorR.setPower(power);
            place = motorR.getBaseCurrentPosition();
        }
        motorR.setPower(0);
    }

    public void rightTurn(double power, int degrees){
        int distance = degreesToDistance(degrees);
        int place = motorL.getBaseCurrentPosition();
        int ticks = place + distanceToTicksR(distance);
        while (RC.l.opModeIsActive() && ticks > place) {
            motorL.setPower(power);
            place = motorL.getBaseCurrentPosition();
        }
        motorL.setPower(0);
    }

    public void shoot(){
        shoot.setPower(75);
    }

    public void feed(){
        load.setPower(0.6);
    }

    public void collect(){
        intake.setPower(75);

    }

}
