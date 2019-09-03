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
}
