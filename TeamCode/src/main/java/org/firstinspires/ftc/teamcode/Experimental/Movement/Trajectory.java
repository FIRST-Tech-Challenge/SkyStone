package org.firstinspires.ftc.teamcode.Experimental.Movement;

import org.firstinspires.ftc.teamcode.Experimental.Units.Vector;

import java.util.ArrayList;

public class Trajectory {
    private ArrayList<Vector> vectors;
    private Vector startingVec;

    public Trajectory(Vector startingVec, ArrayList<Vector> vectors){
        this.vectors = vectors;
        this.startingVec = startingVec;
    }

    public ArrayList<Vector> getVectors() { return vectors; }
    public Vector getStartingVector() { return startingVec; }
}
