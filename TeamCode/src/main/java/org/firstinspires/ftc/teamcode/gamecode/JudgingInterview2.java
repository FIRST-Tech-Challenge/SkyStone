package org.firstinspires.ftc.teamcode.gamecode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.LinearTeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Fermion;
import org.firstinspires.ftc.teamcode.robots.Robot;

/*
 * Created by theon on 2018-03-01.
 */

@TeleOp
@Disabled
public class JudgingInterview2 extends LinearTeleOpMode implements TextToSpeech.OnInitListener{


    TextToSpeech text;

    @Override
    public void loopOpMode() {

        if (joy1.buttonA()){
            text.speak("Hello my name is Felix.", TextToSpeech.QUEUE_FLUSH, null);
            while(text.isSpeaking()) {
                sleep(1);
            }
        }

        if (joy1.buttonB()) {
            text.speak("Thanks Emily!  At the beginning of the year, this crazy group of girls wanted me to drive up on the balancing stone, so I did.  With the help of my four inch Tetrix wheels and omni wheel for steering, and a 3:2 gear ratio for speed, it’s not too hard to get on the balancing stone.  You’ll notice, even if a chain falls off, I keep driving and usually can still balance.\n" + "I’ve gone through several iterations of my glyph collection mechanism, starting with a set of servo-controlled hands, which Eyela is holding up now. \n", TextToSpeech.QUEUE_FLUSH, null);
            while (text.isSpeaking()) {
                sleep(1);
            }

            // Aila holds up the previous versions of hands
            sleep(2000);

            text.speak("Over Christmas, Ines learned how to use CAD, so she could redesign the drive base to make room for my new wheeled intake that was done in CAD too!  So, my intake consists of eight green wheels, geared at a 2:1 ratio, which can collect glyphs even at an angle.  This is all mounted on a linear slide, pulled up and down by string.  Eyela, please feed me a glyph!  It’s behind you! Ya! There you go", TextToSpeech.QUEUE_FLUSH, null);
            while (text.isSpeaking()) {
                sleep(1);
            }

            // Felix intakes a wheel then lifts it up, then spits it out
            // speak, do stuff, sleep, while is speaking
            sleep(4000);

            text.speak("Wake up Ines, we’re almost done.\n" +
                    "\n" +
                    "Over here, you’ll see my jewel flickrs, with a colour sensor on the end. I use those to figure out  the jewel configuration.\n", TextToSpeech.QUEUE_FLUSH, null);
            while (text.isSpeaking()) {
                sleep(1);
            }

            // jewel flickers move down and back up
            sleep(2000);

            text.speak("And these screws, sticking out on my base, line up with the nuts on the balancing stone.  Humans aren’t all that accurate, so these screws help them put me in the same position at the start of every match.\n" +
                    "\n" +
                    "Now, Eyela will tell you more about how my head thinks.\n", TextToSpeech.QUEUE_FLUSH, null);
            while (text.isSpeaking()) {
                sleep(1);
            }
        }

    }

    @Override
    public void stopOpMode() {
        text.shutdown();
    }

    @Override
    public void onInit(int status) {
        RC.t.addData("Talking", "Enabled");
    }

    @Override
    public void initialize() {
        text = new TextToSpeech(RC.c(), this);
        /*fermion = new Fermion(true);
        fermion.stop();
        fermion.startShooterControl();*/
        telemetry.update();
    }
}
