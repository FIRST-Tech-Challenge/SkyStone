package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "ARu5ZM3/////AAABmcGiY9yUyEAsnf3dcn+gux+E9X/ji5wR1QEra3aJBAbIFoL8BPmzx+eUt8sZ7bEwE4IRvwNm32oB/EDVFrGwZtkyOiSR+GKIbM+0G5VYQGwoFNxxGwuUrvpKDS3ktLAuUWmZ0/p/f7ZGwr9di1s4JkzDwr9Hq2B1g16a5F2jf7te3PhLDYaeauXee+WNxv0hp2w64Q91mYwiI+dI9JKsvyruF/FVKVV5Dnf0IGn9mFDGhqSGfkXTOGNpBnjZes5rxndN0PVhvJD+nf1ohsL37m8ORe9zXJqAUJ+vBCaJn7tCtsBJpKgBXYLpWrm05PVXex5cGQsgNc++80BpymExMMCpi6woERNjR86v/cyL+gqg";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}