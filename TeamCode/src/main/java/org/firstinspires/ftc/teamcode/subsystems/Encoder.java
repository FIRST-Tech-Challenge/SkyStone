package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 10/20/2018.
 */
public class Encoder {
    private DcMotor encoder;

    public Encoder(DcMotor encoder){
        this.encoder = encoder;
    }

    public int getEncoderPosition(){
        return encoder.getCurrentPosition();
    }

    public void resetEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
