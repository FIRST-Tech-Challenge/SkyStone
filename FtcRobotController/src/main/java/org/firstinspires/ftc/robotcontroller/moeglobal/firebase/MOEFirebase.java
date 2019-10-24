package org.firstinspires.ftc.robotcontroller.moeglobal.firebase;

import android.content.Context;
import com.google.firebase.FirebaseApp;
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.commands.MOECommands;
import org.firstinspires.ftc.robotcontroller.moeglobal.opmodeloading.OpModesFrame;

public class MOEFirebase {
    public static void init(Context context) {
        FirebaseApp.initializeApp(context);
        MOEStorage.init();
        MOEDatabase.init();
        MOECommands.init();
        MOEConfig.init();
        OpModesFrame.init();
    }
}
