package org.firstinspires.ftc.teamcode.gamecode;

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.LinearTeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Fermion;
import org.firstinspires.ftc.teamcode.robots.Robot;

/**
 * Created by Windows on 2017-02-18.
 */


public class JudgingInterview extends LinearTeleOpMode implements TextToSpeech.OnInitListener{

    TextToSpeech text;
    Fermion fermion;
    
    @Override
    public void loopOpMode() {

        if(joy1.buttonA()) {
            text.speak("Today I am going to describe myself." +
                    "If you look down at my wheels, you will notice that I have 4 mecanum wheels." +
                    "These allow me to drive in any direction and spin simultaneously. This is really helpful for pushing beacons." +
                    "Next you'll notice my collector. Guy, please put a ball in my collector. Don't get too close, I may bite", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            fermion.setCollectorState(Robot.IN);
            clearTimer();
            while (getMilliSeconds() < 4000){
                if (fermion.seesBall()) {
                    fermion.lights.flashState(500);
                }
            }
            fermion.setCollectorState(Robot.STOP);

            text.speak("Now that I have a ball, I can store it in my trough. Once we are ready to shoot, I prime the shooter.", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            text.speak("Then I open the servo to let the ball into the shooter.", TextToSpeech.QUEUE_FLUSH, null);
            fermion.door.goToPos("open");
            sleep(1000);
            fermion.door.goToPos("close");
            sleep(1000);

            text.speak("Now, I will shoot a particle.", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            fermion.shoot();
            fermion.waitForShooterState(Fermion.LOADING);


            text.speak("Now, if Helen will rotate me, you will see my foam button pressers." , TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            text.speak("I also have had some problems with balls getting in front of me so I use my whiskers to clear balls from in front of the beacons", TextToSpeech.QUEUE_FLUSH, null);

            while (opModeIsActive() && text.isSpeaking()){
                idle();
            }
            fermion.clearBall();
            sleep(500);
            fermion.resetBallClearing();
        }

        if(joy1.buttonY()){
            text.speak("Hello, my name is Fermion", TextToSpeech.QUEUE_FLUSH, null);
        }

        if (fermion.seesBall()) {
            fermion.lights.flashState(500);
        }
    }

    @Override
    public void stopOpMode() {
        text.stop();
    }

    @Override
    public void onInit(int status) {
        RC.t.addData("Talking", "Enabled");
    }

    @Override
    public void initialize() {
        text = new TextToSpeech(RC.c(), this);
        fermion = new Fermion(true);
        fermion.stop();
        fermion.startShooterControl();
    }
}
