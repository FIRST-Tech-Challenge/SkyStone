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

package TeamCode.src.main.java.org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Arcade OpMode", group="Iterative Opmode")
public class BasicOpMode_ArcadeTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor bottomleftDrive = null;
    DcMotor bottomrightDrive = null;
    private DcMotor topleftDrive = null;
    DcMotor toprightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        bottomleftDrive  = hardwareMap.get(DcMotor.class, "bottom_left_drive");
        bottomrightDrive = hardwareMap.get(DcMotor.class, "bottom_right_drive");
        topleftDrive = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        bottomleftDrive.setDirection(DcMotor.Direction.REVERSE);
        bottomrightDrive.setDirection(DcMotor.Direction.FORWARD);
        topleftDrive.setDirection(DcMotor.Direction.FORWARD);
        toprightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double bottomleftpower, bottomrightpower, topleftpower, toprightpower;


        double angle = Math.atan((-1.0*gamepad1.left_stick_y)/(gamepad1.left_stick_x));

        //Tangent Inverse only goes from -pi/2 to pi/2, so I have to add some test cases to make sure
        //the angle is correct

        if(gamepad1.left_stick_x < 0 && -1.0*gamepad1.left_stick_y < 0){ //3rd Quadrant
            angle = Math.atan((-1.0*gamepad1.left_stick_y)/(gamepad1.left_stick_x))+Math.PI;
        }
        else if(-1*gamepad1.left_stick_y == 0 && gamepad1.left_stick_x < 0){angle = Math.PI;}
        else if(-1*gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0){
            angle = Math.atan((-1.0*gamepad1.left_stick_y)/(gamepad1.left_stick_x))+Math.PI;
        }
        else if(gamepad1.left_stick_x == 0 && -1.0*gamepad1.left_stick_y > 0){angle = 1.5*Math.PI;}
        else if(-1.0*gamepad1.left_stick_y > 0 && gamepad1.left_stick_x > 0){angle += 2.0*Math.PI;}

        angle = Math.toDegrees(angle); //Changes angle from radians to degrees

        // Not Tank Mode uses two sticks to control.
        // - This requires no math (bet).
        double[] arr = new double[4];

        if(angle == 0){arr = new double[]{1, -1, -1, 1};}
        else if(angle > 0 && Math.toDegrees(angle) < 90){arr = new double[]{1, angle/(Math.PI/4)-1, angle/(Math.PI/4)-1, 1};}
        else if(angle == 90){arr = new double[]{1, 1, 1, 1};}
        else if(angle > 90 && angle < 180){arr = new double[]{3-angle/45, 1, 1, 3-angle/45};}
        else if(angle == 180){arr = new double[]{-1, 1, 1, -1};}
        else if(angle > 180 && angle < 270){arr = new double[]{-1, 5-angle/45, 5-angle/45, -1};}
        else if(angle == 270){arr = new double[]{-1, -1, -1, -1};}
        else if(angle > 270 &&  gamepad1.left_stick_y> 300){arr = new double[]{7-angle/45, -1, -1, 7-angle/45};}
        //
        double bottomleftPower  = arr[2];
        double bottomrightPower = arr[3];
        double topleftPower = arr[0];
        double toprightPower = arr[1];

        // Send calculated power to wheels
        bottomleftDrive.setPower(bottomleftPower);
        bottomrightDrive.setPower(bottomrightPower);
        topleftDrive.setPower(topleftPower);
        toprightDrive.setPower(toprightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "tleft (%.2f), tright (%.2f), bleft (%.2f), bright (%.2f), ANGLE (%.2f), X (%.2f), Y (%.2f)",
                topleftPower, toprightPower, bottomleftPower, bottomrightPower, angle,-1.0*gamepad1.left_stick_x, -1.0*gamepad1.left_stick_y);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}