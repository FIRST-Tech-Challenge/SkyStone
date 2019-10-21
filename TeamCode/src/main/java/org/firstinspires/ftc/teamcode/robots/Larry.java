package org.firstinspires.ftc.teamcode.robots;

import android.hardware.Sensor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.OnBoardSensorManager;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.OnBoardSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by FIXIT on 15-08-21.
 */
public class Larry extends Robot {

    private Motor stomper;
    public Motor scissor;

    public FXTCRServo flag;
    public FXTServo latch;
    public FXTServo autoball;
    public FXTServo gate;

    public static final int LOW_GOAL = 950;
    public static final int MED_GOAL = 1600;
    public static final int HIGH_GOAL = 2700;
    public static final int CENTRE_GOAL = 3350;

    private OnBoardSensorManager fxt;

    public OnBoardSensor gyro;

    public Larry() {

        super();

        fxt = new OnBoardSensorManager();

        scissor = new Motor("scissor");
        stomper = new Motor("stomper");
        stomper.setReverse(true);

        flag = new FXTCRServo("flag");

        latch = new FXTServo("latch");
        autoball = new FXTServo("autoball");
        gate = new FXTServo("gate");

        initializeSensors();

        initializeServos();
    }

    public Larry(Motor driveL, Motor driveR) {
        super(driveL, driveR);

        scissor = new Motor("scissor");
        stomper = new Motor("stomper");
        stomper.setReverse(true);

        flag = new FXTCRServo("flag");
        latch = new FXTServo("latch");
        autoball = new FXTServo("autoball");
        gate = new FXTServo("gate");

        initializeSensors();

        initializeServos();
    }

    private void initializeSensors() {

        gyro = new OnBoardSensor(Sensor.TYPE_GYROSCOPE, fxt);

    }

    private void initializeServos() {

        autoball.addPos("up", 0.58);
        latch.addPos("up", 0.66);
        gate.addPos("open", 0.11);
//        flag.addMotion("up", -566);

        autoball.addPos("down", 0.03);
        latch.addPos("down", 0.41);
        gate.addPos("close", 0.53);
//        flag.addMotion("down", 566);
    }

    public void setHeight(int position){
        if(position > 0) scissorUp(); else scissorDown();
        wait(Math.abs(position));
        scissorStop();
    }

    public void setServosToDefault() {
        autoball.goToPos("up");
        latch.goToPos("up");
        gate.goToPos("close");
    }

    //methods for the scissor lift

    public void scissorUp() {
        scissor.setPower(1.0);
    }

    public void scissorDown() {
        scissor.setPower(-0.5);
    }

    public void scissorStop() {
        scissor.stop();
    }


    public void score(int level) {
        stomperDown();
        wait(1000);
        stomperStop();
        wait(500);
        setHeight(level);
        wait(100);
        gate.goToPos("open");
        wait(1000);
        gate.goToPos("close");
        wait(100);
        setHeight(-level);
        stomperUp();
        wait(450);
        stomperStop();
    }

    //methods for the stomper

    public void stomperUp() {
        stomper.setPower(0.15f);
    }

    public void stomperDown() {
        stomper.setPower(-0.13f);
    }

    public void stomperStop() {
        stomper.stop();
    }

    public void weightedTurnR (int time) {
        motorL.setPower(0.3);
        motorR.setPower(0.1);
        wait(time);
        stop();
    }

    //gyro doesn't always work, so we're not using it

    public void gyroTurnL (double deg, double speed) {

        double targetRadians = Math.toRadians(deg);
        double currentRadians = 0;

        turnL(speed);

        long timer = System.currentTimeMillis();

        while (currentRadians < targetRadians && ((LinearOpMode) RC.o).opModeIsActive()) {

            currentRadians += Math.abs(gyro.getValues()[1]) * ((System.currentTimeMillis() - timer) / 1000.0);
            timer = System.currentTimeMillis();

        }

        stop();
    }

    public void gyroTurnR (double deg, double speed) {

        double targetRadians = Math.toRadians(deg);
        double currentRadians = 0;

        turnR(speed);

        long timer = System.currentTimeMillis();

        while (currentRadians < targetRadians && ((LinearOpMode) RC.o).opModeIsActive()) {

            currentRadians += Math.abs(gyro.getValues()[1]) * ((System.currentTimeMillis() - timer) / 1000.0);
            timer = System.currentTimeMillis();

        }

        stop();

    }

}
