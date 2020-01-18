package org.firstinspires.ftc.teamcode.DutchFTCCore;


//this class contains all function from the configfile
//go to that file for any changes
public class Optional_functions {
    //Drivedirection speed
    public void drivedirectionspeed(){

        if (Function_config_file.drivedirectionspeed){

            if(Function_config_file.linear){
                //linear function that take dx
            }else if(Function_config_file.quadratic){
                //quadratic functions that takes dx
            }else if(Function_config_file.cubic){
                //cubic function that take dx
            }else{
                //message that says to select a function
            }

        }

    }

}
