package org.firstinspires.ftc.teamcode.PreseasonTest;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;

public class SkystonePosition {
    public static double SKYSTONEPOS;
    public static ArrayList<Block> blocks = new ArrayList<Block>();
    public static ArrayList<Double> centerPos = new ArrayList<Double>();
    public static int INDEX;
    public findLocation(ArrayList<Recognition> recognitions) {
        for (Recognition object : recognitions) {
            if (object.getLabel().equals("Skystone")) {
                blocks.add(new Skystone(object.getLeft(), object.getTop(), object.getRight(), object.getBottom(), "Skystone"));
            } else {
                blocks.add(new Stone(object.getLeft(), object.getTop(), object.getRight(), object.getBottom(), "Stone"));
            }
        }
        for (int i = 0; i < blocks.size(); i++) {
            if(blocks.get(i).label == "Skystone"){
                SKYSTONEPOS = blocks.get(i).centerX;
            }
        }
        for(int i=0; i<blocks.size(); i++){
            centerPos.add(blocks.get(i).centerX);
        }
        Collections.sort(centerPos);

        INDEX = centerPos.indexOf(SKYSTONEPOS) + 1;

        if
    }
}
