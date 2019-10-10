package org.firstinspires.ftc.teamcode.opmodes;
    /* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.components.*;
import android.util.*;

public class AutoStateMachinePattern1 extends BaseStateMachine {

    private final String TAG = "AutoState1";
    public ElapsedTime mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    protected State mCurrentState;    // Current State Machine State.

    @Override
    public void loop() {
        switch (mCurrentState) {
            case STATE_INITIAL:
                Log.d(TAG, "Initial State Begun");
                // Initialize
                mStateTime.reset();
                /*colorSensor = hardwareMap.get(ColorSensor.class, "color_Sensor");
                Log.d("Color Sensor Initialized");
                this.motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
                this.motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
                this.motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
                this.motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
                Log.d("DriveMotors - Initialized");*/
                break;

            case STATE_FIND_SKYSTONE:
                Log.d(TAG, "Finding/Looking for Skystone");
                // Strafe towards line
                // Identify SkyStone
                /*
                if(vuforia circumstances met && !isYellow()){
                    newState(State.STATE_GRAB_STONE);
                    //STATE_GRAB_STONE
                }
                // If we can't see it after 3 feet, start driving  until we do
                else{

                }
                */
                break;

            case STATE_GRAB_STONE:


                // Grab the stone and slurp it into the machine
                newState(State.STATE_DELIVER_STONE);
                break;

            case STATE_DELIVER_STONE:
                // Drive with stone to the foundation
                // Go under bridge
                newState(State.STATE_DEPOSIT_STONE);
                break;

            case STATE_FIND_STONE:
                // Find a stone using TensorFlow
                newState(State.STATE_GRAB_STONE);
                break;

            case STATE_PARK_AT_LINE:
                // Find the line
                // Park
                break;

            case STATE_DEPOSIT_STONE:
                // Put stone down on foundation
                newState(State.STATE_RETURN);
                break;

            case STATE_DRAG_FOUNDATION:
                // Drag foundation out of box
                newState(State.STATE_PARK_AT_LINE);
                break;

            case STATE_RETURN:
                // Go back to block repository
                newState(State.STATE_FIND_SKYSTONE);
                break;
        }
    }
}