package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Crane;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="Single Vuf", group = "basic")

public class singlevalueVuf extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
    private boolean target = false;

    public void identify(){
        for (VuforiaTrackable trackable : rob.allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(rob.robotFromCamera, rob.parameters.cameraDirection);
        }

        rob.targetsSkyStone.activate();
        while (opModeIsActive() && !target) {
            for (VuforiaTrackable trackable : rob.allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    if (trackable.getName().equals("Stone Target")) {
                        telemetry.addData("good", "none");
                        move();
                    } else {
                        telemetry.addData("bad", "none");
                    }
                    break;
                }
            }

            telemetry.update();
        }
    }

    public void move(){
        rob.targetVisible = false;
        for (VuforiaTrackable trackable : rob.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                rob.targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    rob.lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (rob.targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = rob.lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / rob.mmPerInch, translation.get(1) / rob.mmPerInch, translation.get(2) / rob.mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(rob.lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            if(translation.get(1)>0) {
                telemetry.addData("move left", "none");
            }else if(translation.get(1)<0){
                telemetry.addData("move right", "none");
            }else{
                telemetry.addData("move straight", "none");
            }

            if(translation.get(0)<-1) {
                telemetry.addData("move up", "none");
            }else{
                telemetry.addData("move nothing", "none");
            }
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Crane.setupType.camera);
        telemetry.addLine("Start!");
        telemetry.update();

        identify();

        rob.targetsSkyStone.deactivate();

    }
}
