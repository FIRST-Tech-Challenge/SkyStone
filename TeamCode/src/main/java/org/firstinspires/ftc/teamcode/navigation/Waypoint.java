package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.common.math.Point;

import java.util.Arrays;
import java.util.LinkedList;

public class Waypoint extends Point implements Cloneable {

    public double followDistance;

    public Waypoint(Point p, double followDistance) {
        this(p.x, p.y, followDistance);
    }


    // Undeclared variables are null
    public Waypoint(double x, double y, double followDistance) {
        super(x, y);

        this.followDistance = followDistance;
    }

    @Override
    public Waypoint clone() {
        return new Waypoint(x, y, followDistance);
    }

    public static LinkedList<Waypoint> collate(Waypoint... waypoints) {
        return new LinkedList<>(Arrays.asList(waypoints));
    }
}