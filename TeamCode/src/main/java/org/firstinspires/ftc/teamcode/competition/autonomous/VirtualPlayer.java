package org.firstinspires.ftc.teamcode.competition.autonomous;

import org.firstinspires.ftc.teamcode.helperclasses.WayPoint;

import java.util.ArrayList;

/**
 * Pretty much an AI player for auto but it isn't actually AI
 */
public class VirtualPlayer {

    private ArrayList<WayPoint> points = new ArrayList<WayPoint>();

    public VirtualPlayer(){

    }

    public VirtualPlayer(ArrayList<WayPoint> points) {
        this.points = points;
    }

    /* TODO: Add a recording system where we drive the robot in teleop and it places 'WayPoints' every
        amount of time, and after stopping the auto it creates a pure pursuit path from the
        WayPoints we would set
       TODO: Not only would it record the driving, it would record velocities at a faster speed
        which could be converted into bias and turning speeds
       TODO: (Possibly) create a new turning algorithm that better works with the recorded turning
       TODO: Implement a checkpoint system where the robot knows to pause the pursuit and activate
        certain mechanisms or do other tasks
       TODO: Options for how to get checkpoints:
        - Robot has a circle and checkPOINTS need to enter them
        - Robot has a circle and the checkpoint is a circle, they need to intersect
        - Robot has a single point and checkpoints have a radius the robot needs to enter
       TODO: Write all recording data to a human-readable file which can be edited in case we messed
        up a certain points or want to adjust the data in anyway
       TODO: Section off the field into certain parts which will have their own Pursuit and certain
        ways it needs to do something
       TODO: (At the end of a path it could do a certain tasks)
       TODO: Auto-generate alternate routes based on current position if odometry values aren't
        getting closer to the value it needs to
    */
}
