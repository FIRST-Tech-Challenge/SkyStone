package org.clueless.motionplanning.field_positioning.field_position_modules;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.*;

/**
 * Created by DrZ on 12/1/18.
 */

public class Coordinate_min {
    private DistanceSensor  D1, D2, D3 ,D4;
    double Da = D1.getDistance(DistanceUnit.CM);
    double Db = D1.getDistance(DistanceUnit.CM);
    double Dc = D1.getDistance(DistanceUnit.CM);
    double Dd = D1.getDistance(DistanceUnit.CM);
    double arr[] = {Da,Db,Dc,Dd};
    int first, second, arr_size = arr.length;
    public void print2Smallest(int arr[])
    {

        first = second = Integer.MAX_VALUE;
        for (int i = 0; i < arr_size ; i ++)
        {

            if (arr[i] < first)
            {
                second = first;
                first = arr[i];
            }


            else if (arr[i] < second && arr[i] != first)
                second = arr[i];
        }


    }

}
