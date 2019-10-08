package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook {
    Servo hook;

    public Hook(Servo hook) {
        this.hook = hook;
    }

    public Servo getHook() {
        return hook;
    }

    public void setHook(Servo hook) {
        this.hook = hook;
    }

    public void setHookPosition(double power) {
        hook.setPosition(power);
    }

    public void setPower(double power) {
        setHookPosition(power);
    }
}