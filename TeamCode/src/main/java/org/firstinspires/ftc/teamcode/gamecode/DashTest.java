package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.*;

/**
 * Created by Windows on 2017-01-21.
 */
@Disabled
@Autonomous
public class DashTest extends AutoOpMode {

    static final String[] poem = new String[] {

            "Mary had a little lamb,",
            "His fleece was white as snow,",
            "And everywhere that Mary went,",
            "The lamb was sure to go.",
            "",
            "He followed her to school one day,",
            "Which was against the rule,",
            "It made the children laugh and play",
            "To see a lamb at school.",
            "",
            "And so the teacher turned it out,",
            "But still it lingered near,",
            "And waited patiently about,",
            "Till Mary did appear.",
            "",
            "\"Why does the lamb love Mary so?\"",
            "The eager children cry.",
            "\"Why, Mary loves the lamb, you know,\"",
            "The teacher did reply.",
            "",
            ""
    };

        @Override
        public void runOp() throws InterruptedException{

            waitForStart();

            RC.t.speakString("");

            for (String line : poem) {
                RC.t.speakString(line);
                sleep(100);
            }//for

            RC.t.beep(400, 500);

        }//runOp

}//DashTest