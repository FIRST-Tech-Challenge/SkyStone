package org.firstinspires.ftc.teamcode.gamecode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.LinearTeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Armstrong;

@TeleOp
public class JudgingInterviewBuildCopy extends LinearTeleOpMode implements TextToSpeech.OnInitListener{

    TextToSpeech text;
    Armstrong armstrong;
    public void initialize() {

        text = new TextToSpeech(RC.c(), this);
        armstrong = new Armstrong();
        telemetry.addData("status", "initialized");
    }
    @Override


    public void loopOpMode() {
        if (joy1.buttonA()) {
            text.speak("Hello judges!" +
                    "My name is Armstrong, and I’m going to show you my mechanisms."+
                    "First off, my drive base, I zoom around with a six wheel drive base, geared two-to-one." +
                    "My front and back wheels are Omni and my middle are standard tetrix wheels." +
                    "Now you may have guessed already from my name, I have a super strong arm used for lifting myself, I bet Neil Armstrong couldn’t beat my pull-up skills."
                    + "This strength comes from a rack and pinion device geared with a worm gear.", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            armstrong.lifterUp();

            text.speak("And it raises expectations.", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            armstrong.lifterStop();
            //spin


            text.speak("Next my latch. I use a one-eighty servo for this dandy device.", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            armstrong.unlatch();
            text.speak("I use the circular motion and transform it to linear motion." +
                    "It’s pretty effective if I do say so myself.", TextToSpeech.QUEUE_FLUSH, null);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            armstrong.setLatch();


            text.speak("Now to collect minerals. I have here some handy hair scratchers, but not for scratching my beautiful robot hairs.", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                 idle();
            }
            armstrong.armdownslow();
            sleep(1500);
            armstrong.armstop();

            text.speak("These along with my two scoring arms are used to snatch up minerals and score them into the lander.", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            armstrong.armupslow();
            sleep(1500);
            armstrong.armstop();

            text.speak("The arms are geared and use torque to reach either side of Armstrong. Ishaan catch!", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            armstrong.armdownslow();
            sleep(1900);
            armstrong.armstop();

            armstrong.collectServoRightDown();
            armstrong.collectServoLeftDown();
            text.speak(" I can boogie with these chain-driven linear slides to reach far away minerals", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
            armstrong.collectServoRightStop();
            armstrong.collectServoLeftStop();

            text.speak(" I got these groovy wings for sampling as well as this rad wall", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            armstrong.RightSample();
            armstrong.LeftSample();

            sleep(1000);

            text.speak("Also, I deploy a team marker with this one-eighty-servo", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }

            armstrong.markDown();

            text.speak("With that I'm ready to blast into competition at any time!", TextToSpeech.QUEUE_FLUSH, null);
            sleep(50);
            while (opModeIsActive() && text.isSpeaking()) {
                idle();
            }
        }



    }

    @Override

    public void stopOpMode() {text.stop();}
    @Override
    public void onInit(int status) {
        RC.t.addData("Talking", "Enabled");
    }


}
