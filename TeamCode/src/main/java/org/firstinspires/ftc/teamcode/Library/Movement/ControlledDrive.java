/*
 * Drive exact based on the IMU and/or Encoders
 * partly based on org.firstinspires.ftc.robotcontroller.external.samples.PushbbotAutoDriveByEncoder_Linear
 */

package org.firstinspires.ftc.teamcode.Library.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareOmniTest;
import org.firstinspires.ftc.teamcode.Library.OmniWheel;

import java.util.function.Supplier;

public class ControlledDrive {
    static final double     COUNTS_PER_MOTOR_REV    = 5264.0 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CMS      = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    HardwareOmniTest hwMap;
    HardwareMap hardwareMap;

    public ControlledDrive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.hwMap = new HardwareOmniTest(hardwareMap);
        this.hwMap.init();
    }

    public void driveDistance(double distanceForward, double distanceSideways) {

        double maxDistance = Math.max(distanceForward, distanceSideways);

        double[] wheelSpeeds =  OmniWheel.calculate(WHEEL_DIAMETER_CMS/2, 38, 24, distanceForward/maxDistance ,distanceSideways/maxDistance, 0);

        double[] targets = {hwMap.motor_front_left.getCurrentPosition() + wheelSpeeds[0]*(COUNTS_PER_CM*maxDistance), hwMap.motor_front_right.getCurrentPosition() + wheelSpeeds[1]*(COUNTS_PER_CM*maxDistance), hwMap.motor_back_left.getCurrentPosition() + wheelSpeeds[2]*(COUNTS_PER_CM*maxDistance), hwMap.motor_back_right.getCurrentPosition() + wheelSpeeds[3]*(COUNTS_PER_CM*maxDistance)};

    }

    // driveCondition(1,1, () -> button.getState() == false);
    public void driveCondition(double speedForward, double speedSideways, Supplier<Boolean> condition) {
        while (condition.get()) {
            //drive
        }
    }

    public void rotate(double degree) {

    }
}