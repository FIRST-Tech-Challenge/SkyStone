package org.firstinspires.ftc.teamcode.DutchFTCCore;


//this class contains all function from the configfile
//go to that file for any changes
public class Optional_functions {
    //Drivedirection speed
    public void drivedirectionspeed(){

        if (Function_config_file.drivedirectionspeed){
            double dds = Function_config_file.drivespeed;
            if(Function_config_file.linear){
                dds = dds + Function_config_file.dx;
            }else if(Function_config_file.quadratic){
                dds = Math.pow(Math.sqrt(dds) + Function_config_file.dx, 2);
            }else if(Function_config_file.cubic){
                dds = Math.pow(Math.cbrt(dds) + Function_config_file.dx, 3);
            }else{
                //message that says to select a function
            }

        }

    }

}
