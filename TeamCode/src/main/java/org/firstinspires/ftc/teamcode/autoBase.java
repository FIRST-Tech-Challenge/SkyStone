package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Type;

public abstract class autoBase extends LinearOpMode {
    TypexChart chart = new TypexChart();
    Orientation lastAngles = new Orientation();
    PIDController pidRotate, pidDrive;
    double globalAngle, power = .30, correction, rotation;



    @Override
    public void runOpMode() throws InterruptedException {
        chart.init(hardwareMap);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();


        correction = pidDrive.performPID(getAngle());
    }


    public void raiseDL() {
        chart.hookLeft.setPosition(0.9);
        chart.hookRight.setPosition(0.9);
    }

    public void rest() {
        chart.TL.setPower(0);
        chart.TR.setPower(0);
        chart.BL.setPower(0);
        chart.BR.setPower(0);
    }

    public void dropDL() {
        chart.hookLeft.setPosition(0.1);
        chart.hookRight.setPosition(0.1);
    }

    public boolean tripWireActive(double triggerDist) {
        if (chart.distanceSensor.getDistance(DistanceUnit.CM) < triggerDist) {
            return true;
        } else {
            return false;
        }
    }

    public void wait(double seconds) {
        ElapsedTime Intruntime = new ElapsedTime();
        Intruntime.reset();
        while (opModeIsActive() && Intruntime.seconds() < seconds) {
            telemetry.addData("Status: ", "Executing the current Phase");
            telemetry.update();
        }
    }

    public void wait(double seconds, String phase) {
        ElapsedTime Intruntime = new ElapsedTime();
        Intruntime.reset();
        while (opModeIsActive() && Intruntime.seconds() < seconds) {
            telemetry.addData("Status: ", phase);
            telemetry.update();
        }
        //sleep(500);
    }

    public void strafeCorrection(){
        double pulseStrength = 0.05;

        resetAngle();

        if (getAngle()>5){
            chart.TL.setPower(chart.TL.getPower() + pulseStrength);
        }
        if (getAngle()<-5) {
            chart.BL.setPower(chart.BL.getPower() + pulseStrength);
        }
        else {
            pulseStrength = 0;
        }
    }

    private void resetAngle() {
        lastAngles = chart.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double joltControl() {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        if (runtime.seconds() < 1.2) {
            return 0.05;
        } else {
            return 0.0;
        }
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = chart.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                chart.TL.setPower(power);
                chart.BL.setPower(power);
                chart.TR.setPower(-power);
                chart.BR.setPower(-power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                chart.TL.setPower(-power);
                chart.BL.setPower(-power);
                chart.TR.setPower(power);
                chart.BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                chart.TL.setPower(-power);
                chart.BL.setPower(-power);
                chart.TR.setPower(power);
                chart.BR.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        chart.TL.setPower(0);
        chart.TR.setPower(0);
        chart.BL.setPower(0);
        chart.BR.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void goForward(){
        chart.TL.setPower(-(power - correction));
        chart.BL.setPower(-(power - correction));
        chart.TR.setPower(-(power + correction));
        chart.BR.setPower(-(power + correction));
    }

    public void goBack() {
        chart.TL.setPower((power - correction));
        chart.BL.setPower((power - correction));
        chart.TR.setPower((power + correction));
        chart.BR.setPower((power + correction));
    }
}
