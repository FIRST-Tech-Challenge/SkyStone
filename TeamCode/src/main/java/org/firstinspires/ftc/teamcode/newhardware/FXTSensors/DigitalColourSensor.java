package org.firstinspires.ftc.teamcode.newhardware.FXTSensors;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTDevice;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Windows on 2016-04-02.
 */
public class DigitalColourSensor implements FXTDevice {
    DigitalChannel input;
    DigitalChannel s2;
    DigitalChannel s3;
    public static final int RED = 0;
    public static final int GREEN = 2;
    public static final int BLUE = 1;
    public static final int NONE = 3;
    int counter = 0;
    int count = 0;
    int RGB = RED;
    boolean recording = true;
    int[] colours = new int[4];

    public DigitalColourSensor(String input, String s2, String s3) {
        this.input = RC.h.digitalChannel.get(input);
        this.s2 = RC.h.digitalChannel.get(s2);
        this.s3 = RC.h.digitalChannel.get(s3);
        this.s2.setMode(DigitalChannelController.Mode.OUTPUT);
        this.s3.setMode(DigitalChannelController.Mode.OUTPUT);
    }

    public void check() {
        count = 0;
        boolean lastDigitalRead = true;
        for (int j = 0; j < 80000; j++) {
            boolean digitalReadValue = input.getState();
            if (!lastDigitalRead && digitalReadValue) {
                count++;
            }
            counter++;
            lastDigitalRead = digitalReadValue;
        }
        Log.i("RGB", RGB + "" + count);
        if (recording) {
            colours[RGB] = count;
        }
        counter = 0;
        count = 0;
        setColorFilter((RGB + 1) % 4);
    }

    public int getColour() {
        recording = false;
        if (colours[RED] > colours[BLUE] && colours[RED] > colours[GREEN] && colours[RED] > colours[NONE]) {
            recording = true;
            return RED;
        }
        if (colours[BLUE] > colours[RED] && colours[RED] > colours[GREEN] && colours[RED] > colours[NONE]) {
            recording = true;
            return BLUE;
        }
        if (colours[GREEN] > colours[BLUE] && colours[GREEN] > colours[RED] && colours[GREEN] > colours[NONE]) {
            recording = true;
            return GREEN;
        }
        if (colours[NONE] > colours[BLUE] && colours[NONE] > colours[GREEN] && colours[NONE] > colours[RED]) {

            recording = true;
            return NONE;
        }
        recording = true;
        return 5;
    }
    public void setColorFilter(int colour) {
        switch (colour) {
            case BLUE:
                s2.setState(false);
                s3.setState(true);
                break;
            case RED:
                s2.setState(false);
                s3.setState(false);
                break;
            case GREEN:
                s2.setState(true);
                s3.setState(true);
                break;
            case NONE:
                s2.setState(true);
                s3.setState(false);
        }
        RGB = colour;
    }

    public int getRed() {
        setColorFilter(RED);
        return colours[RED];
    }

    public int getClear() {
        setColorFilter(NONE);
        return colours[NONE];
    }

    public int getBlue() {
        setColorFilter(BLUE);
        return colours[BLUE];
    }

    public int getGreen() {
        setColorFilter(GREEN);
        return colours[GREEN];
    }
}
/*
void setup()
{
  pinMode(OUT, INPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  digitalWrite(S0, LOW);  // OUTPUT FREQUENCY SCALING 2%
  digitalWrite(S1, HIGH);


  Serial.begin(9600);
  Serial.println("Starting...");

  digitalWrite(VCC, HIGH);
  delay(100);

  for(int i=0; i<3; i++)
  {
    Select_Filters(i);
    int lastDigitalRead = HIGH;
    for(int j=0; j<20000; j++)
    {
      int digitalReadValue = digitalRead(OUT);
      if (lastDigitalRead == LOW && digitalReadValue == HIGH)
      {
        count++;
      }
      counter++;
      lastDigitalRead = digitalReadValue;
    }

    scaleFactor[i] = 255.0/count;

    Serial.print("count: ");
    Serial.println(count);
    Serial.print("counter: ");
    Serial.println(counter);
    Serial.print("scaleFactor: ");
    Serial.println(scaleFactor[i],9);
    counter=0;
    count=0;
  }

  digitalWrite(VCC, LOW);

}

void loop()
{
  Serial.println("loop");

  digitalWrite(VCC, HIGH);
  delay(100);

  for(int i=0; i<3; i++)
  {
    RGB=i;
    Select_Filters(i);
    count=0;
    int lastDigitalRead = HIGH;
    for(int j=0; j<20000; j++)
    {
      int digitalReadValue = digitalRead(OUT);
      if (lastDigitalRead == LOW && digitalReadValue == HIGH)
      {
        count++;
      }
      counter++;
      lastDigitalRead = digitalReadValue;
    }

    Serial.print("value: ");
    Serial.println(scaleFactor[i]*count);
    Serial.print("counter ");
    Serial.println(counter);
    counter=0;
    count=0;
  }

  digitalWrite(VCC, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);

  delay(3000);
  digitalWrite(S1, HIGH);
}

void Select_Filters(int RGB)
{
  switch(RGB)
  {
    case Filters_R:          //Red
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    Serial.println("-----select Red color");
    break;

    case Filters_G:          //Green
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    Serial.println("-----select Green color");
    break;

    case Filters_B:          //Blue
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    Serial.println("-----select Blue color");
    break;

    default:                  //Clear(no filter)
    digitalWrite(S2, HIGH);
    digitalWrite(S3, LOW);
    Serial.println("-----no filter");
    break;
  }
}
 */