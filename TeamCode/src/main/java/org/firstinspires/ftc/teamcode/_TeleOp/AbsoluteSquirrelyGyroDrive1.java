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

package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

/*
 * TeleOp Mode
 * Enables control of the robot via the gamepad such that the robot moves in the
 * absolute direction and speed indicated by the left joystick, assuming the game console is
 * aligned with the robot when the mode is initiated, and the robot maintains an orientation
 * aligned with the direction of the right joystick..
 */

@TeleOp(name="AbsoluteSquirrelyGyroDrive1", group="Test")  // @Autonomous(...) is the other common choice
//@Disabled
public class AbsoluteSquirrelyGyroDrive1 extends OpMode {

	DcMotor motorFrontRight;
	DcMotor motorFrontLeft;
	DcMotor motorBackRight;
	DcMotor motorBackLeft;

	boolean bDebug = false;

	AutoLib.SquirrelyGyroTimedDriveStep mStep;

	ModernRoboticsI2cGyro mGyro;            // gyro to use for heading information
	SensorLib.CorrectedMRGyro mCorrGyro;    // gyro corrector object

	DcMotor mMotors[];

	/**
	 * Constructor
	 */
	public AbsoluteSquirrelyGyroDrive1() {

	}


	@Override
	public void init() {

		/*
		 * For this test, we assume the following,
		 *   There are four motors
		 *   "fl" and "bl" are front and back left wheels
		 *   "fr" and "br" are front and back right wheels
		 */
		try {
			AutoLib.HardwareFactory mf = null;
			final boolean debug = true;
			if (debug)
				mf = new AutoLib.TestHardwareFactory(this);
			else
				mf = new AutoLib.RealHardwareFactory(this);

			// get the motors: depending on the factory we created above, these may be
			// either dummy motors that just log data or real ones that drive the hardware
			// assumed order is fr, br, fl, bl
			mMotors = new DcMotor[4];
			mMotors[0] = mf.getDcMotor("fr");
			mMotors[1] = mf.getDcMotor("br");
			(mMotors[2] = mf.getDcMotor("fl")).setDirection(DcMotor.Direction.REVERSE);
			(mMotors[3] = mf.getDcMotor("bl")).setDirection(DcMotor.Direction.REVERSE);

			// get hardware gyro
			mGyro = (ModernRoboticsI2cGyro) mf.getGyro("gyro");

			// wrap gyro in an object that calibrates it and corrects its output
			mCorrGyro = new SensorLib.CorrectedMRGyro(mGyro);
			mCorrGyro.calibrate();
		}
		catch (IllegalArgumentException iax) {
			bDebug = true;
		}

		// create a Step that we will use in teleop mode
		mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, mCorrGyro, null, mMotors, 0, 10000, false);
	}


	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		// motion direction is on the left stick
		float dx = gamepad1.left_stick_x;
		float dy = -gamepad1.left_stick_y;	// y is reversed :(

		// power is the magnitude of the direction vector
		double power = Math.sqrt(dx*dx + dy*dy);
		mStep.setPower((float) power);

		// we don't have a valid direction when inputs are "zero"
		final double MIN_INPUT = 0.1;
		if (power > MIN_INPUT) {
			// direction angle of stick >> the direction we want to move
			double direction = Math.atan2(-dx, dy);    // stick angle: zero = +y, positive CCW, range +-pi
			direction *= 180.0 / Math.PI;        // radians to degrees
			mStep.setDirection((float) direction);
		}

		// vehicle heading (orientation) is on the right stick
		float hx = gamepad1.right_stick_x;
		float hy = -gamepad1.right_stick_y;    // y is reversed :(

		double hMag = Math.sqrt(hx*hx + hy*hy);
		if (hMag > MIN_INPUT) {
			// direction angle of stick >> the direction we want to face
			double heading = Math.atan2(-hx, hy);    // stick angle: zero = +y, positive CCW, range +-pi
			heading *= 180.0 / Math.PI;        // radians to degrees
			mStep.setHeading((float) heading);
		}

		// run the control step
		mStep.loop();
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
	}

}
