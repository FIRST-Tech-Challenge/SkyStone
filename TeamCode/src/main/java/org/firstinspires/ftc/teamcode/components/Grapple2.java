/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
public class Grapple2 extends BotComponent {
    //SERVOS ARE OPPOSITE, USE MIRRORED VALUES
    public Servo grapple1 = null;
    public Servo grapple2 = null;

    /*
    instead of using this method, using bare numbers is recommended
    double SERVO_DOWN_POSITION = 0.77;
    double SERVO_UP_POSITION = 0.1;
    double SERVO2_DOWN_POSITION = 0.77;
    double SERVO2_UP_POSITION = 0.1;
    */
    public Grapple2() {
    }

    public Grapple2(Logger aLogger, OpMode aOpMode, String servoName, String servoName2) {
        super(aLogger, aOpMode);
        //define and initialize motors
        logger.logDebug("initservo", "IamWalrus");
        grapple1 = initServo(servoName, 0.2);
        grapple2 = initServo(servoName2, 0.77);
        if (grapple1 != null && grapple2 != null) {
            isAvailable = true;
        }

        logger.logInfo("Grapple", "isAvailable: %b", isAvailable);
    }

    //Functions for GRAPPLE 1
    //GRAPPLE ONE DOWN POSITION STARTS AT AROUND 1 (use 0.8 - 0.7 values)
    public void grapple1Down() {
        logger.logDebug("servoMoveDown", "walrus");
        grapple1.setPosition(0.77);
    }

    //GRAPPLE ONE UP POSITION STARTS AT AROUND 0 (use 0.2 - 0.3 range values)
    public void setGrapple1Up() {
        grapple1.setPosition(0.3);
    }

    //Functions for GRAPPLE 2
    //GRAPPLE TWO DOWN POSITION STARTS AT AROUND 0 (use 0.2 - 0.3 range values)
    public void setGrapple2Down(){
        grapple2.setPosition(0.2);
    }

    //GRAPPLE TWO UP POSITION STARTS AT AROUND 1 (use 0.8 - 0.7 range values)
    public void setGrapple2Up(){
        grapple2.setPosition(0.77);
    }
}


