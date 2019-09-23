package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.westtorrancerobotics.lib.MecanumController;

public class Robot {
    public final ElapsedTime runtime;

    public final DriveTrain driveTrain;
    public final FoundationGrabber foundationGrabber;
    public final Lift lift;
    public final StoneManipulator stoneManipulator; // includes capstone
    public final Camera camera;

    public Robot(HardwareMap hardwareMap) {

        runtime = new ElapsedTime();

        driveTrain = new DriveTrain(hardwareMap);
        foundationGrabber = new FoundationGrabber(hardwareMap);
        lift = new Lift(hardwareMap);
        stoneManipulator = new StoneManipulator(hardwareMap);
        camera = new Camera(hardwareMap);

    }

}
