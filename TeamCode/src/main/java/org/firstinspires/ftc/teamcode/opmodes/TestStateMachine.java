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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
// A2818_StateMachine.java
//------------------------------------------------------------------------------
// Extends the OpMode class to provide a Example Autonomous code
//------------------------------------------------------------------------------
    /* This opMode does the following steps:
     * 0) Wait till the encoders show reset to zero.
     * 1) Drives to the vicinity of the beacon using encoder counts
     * 2) Use the Legacy light sensor to locate the white line
     * 3) Tracks the line until the wall is reached
     * 4) Pushes up against wall to get square using constant power an time.
     * 5) Deploys the Climbers using the servo
     * 6) Drives to the Mountain using encoder counts
     * 7) Climbs the Mountain using constant speed and time
     * 8) Stops and waits for end of Auto
     *
     * The code is executed as a state machine.  Each "State" performs a specific task which takes time to execute.
     * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on to next state
     */

    public class TestStateMachine extends OpMode {
        // A list of system States.
        private enum State {
            STATE_INITIAL,
            STATE_FIND_SKYSTONE,
            STATE_DELIVER_STONE,
            STATE_GRAB_STONE,
            STATE_FIND_STONE,
            STATE_PARK_AT_LINE,
            STATE_DEPOSIT_STONE,
            STATE_DRAG_FOUNDATION,
            STATE_SCOOP_STONE,
        }

        //--------------------------------------------------------------------------
        // Robot device Objects
        //--------------------------------------------------------------------------


        // Loop cycle time stats variables
        public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

        private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

        private State       mCurrentState;    // Current State Machine State.


        //--------------------------------------------------------------------------
        // init
        //--------------------------------------------------------------------------
        @Override
        public void init() {
            // Initialize class members
        }

        //--------------------------------------------------------------------------
        // loop
        //--------------------------------------------------------------------------
        // @Override
        public void init_loop() {
        }

        //--------------------------------------------------------------------------
        // start
        //--------------------------------------------------------------------------
        @Override
        public void start() {

        }

        //--------------------------------------------------------------------------
        // loop
        //--------------------------------------------------------------------------
        @Override
        public void loop() {
            // Send the current state info (state and time) back to first line of driver station telemetry.

            // Execute the current state.  Each STATE's case code does the following:
            // 1: Look for an EVENT that will cause a STATE change
            // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
            //   else
            // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
            //
            switch (mCurrentState) {
                case STATE_INITIAL:
                    // Initialize
                    break;

                case STATE_FIND_SKYSTONE:
                    // Strafe towards line
                    // Identify SkyStone
                    // If we can't see it after 3 feet, start driving  until we do
                    break;

                case STATE_GRAB_STONE:

                    break;

                case STATE_DELIVER_STONE:

                    break;

                case STATE_FIND_STONE:

                    break;

                case STATE_PARK_AT_LINE:

                    break;

                case STATE_DEPOSIT_STONE:

                    break;

                case STATE_DRAG_FOUNDATION:

                    break;

                case STATE_SCOOP_STONE:

                    break;
            }
        }

        //--------------------------------------------------------------------------
        // stop
        //--------------------------------------------------------------------------
        @Override
        public void stop() {
        }


    }

