package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class Rectangle{
    private double length;
    private double width;
    public Rectangle(double length, double width){
      this.length = length;
      this.width = width;
    }
    public double getPerimeter(){
      return length + width;
    }
    public double getArea(){
      return length*width;
    }
  }