package org.firstinspires.ftc.teamcode.BlockDetectionTest;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SkystonePosition {
    /*
    This program uses the x-coordinate of the center of the bounding box to determine the position
    of the Skystone. It takes a list of recognitions and sorts the centerX coordinates in an ArrayList
    The program then returns an enum (BLOCKPOS.(LEFT/MIDDLE/RIGHT)) based on the index of the Skystone
    centerX pos.
     */

    public BLOCKPOS findLocation(List<Recognition> recognitions) {
        double SKYSTONEX = 0;
        ArrayList<Block> blocks = new ArrayList<Block>();
        ArrayList<Double> centerPos = new ArrayList<Double>();
        int INDEX = -2;
        BLOCKPOS position = BLOCKPOS.NONE;
        int numObjects = 0;

        for (Recognition object : recognitions) {
            if (object.getLabel().equals("Skystone")) {
                blocks.add(new Skystone(object.getLeft(), object.getTop(), object.getRight(), object.getBottom(), "Skystone"));
            } else {
                blocks.add(new Stone(object.getLeft(), object.getTop(), object.getRight(), object.getBottom(), "Stone"));
            }
        }
        numObjects = recognitions.size();

        for (int i = 0; i < blocks.size(); i++) {
            if(blocks.get(i).label == "Skystone"){
                SKYSTONEX = blocks.get(i).centerX;
            }else{
                SKYSTONEX = -1;
            }
        }

        for(int i=0; i<blocks.size(); i++){
            centerPos.add(blocks.get(i).centerX);
        }

        Collections.sort(centerPos);

        INDEX = centerPos.indexOf(SKYSTONEX);

        if (numObjects == 2) {
            if(INDEX == 0){
                position = BLOCKPOS.LEFT;
            }else if(INDEX == 1){
                position = BLOCKPOS.MIDDLE;
            }else{
                position = BLOCKPOS.RIGHT;
            }
        }
        else{
            if(INDEX == 0){
                position = BLOCKPOS.LEFT;
            }else if(INDEX == 1){
                position = BLOCKPOS.MIDDLE;
            }else if(INDEX == 2){
                position = BLOCKPOS.RIGHT;
            }
        }

        return position;
    }
}
