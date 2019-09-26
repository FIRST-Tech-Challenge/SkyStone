package org.firstinspires.ftc.teamcode;

public class AutoInstructions {
}
/*Configure all your hardware. You should be using a Linear OpMode (you can erase the variables and contents of the methods in Linear OpModes that come with the SDK to get started). Example:
motor_a = hardwareMap.dcMotor.get("left");
Get the robot to move. Whether this entails encoders, time, or anything, the basis of autonomous lies on movement. Example:
motor_a.setPower(1); motor_b.setPower(2);
Get your sensors working, No matter which sensors you use, make sure you can get the data you need from them reliably. Example:
rateOfRotation = gyro.getRotation();
Make judgements. Figure out what values mean what – now that you can poll for data, what data should signal you to do what? Example:
if(irSeeker.signalDetected()) beaconFound = true;
Piece the movement together with the judgements and TEST. Testing is the most important part. Example: On the Lancers, we have many spreadsheets full of test data, running at least 10 trials for each form of autonomous and testing each possible scenario.
If something doesn’t work, take a different approach. Example: In the Cascade Effect 2014-15 season, the IR Beacons used were circular; they returned different data. In addition, the IR sensors found it difficult to differentiate between diffeent positions.
 For this reason, we weighted the IR’s judgement as 25% reliable, and introduced a sonar that could judge 75% reliably. We then used the weighted data to figure out what position was most probable.
 */