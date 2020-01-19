package org.firstinspires.ftc.teamcode.DutchFTCCore;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;
import java.util.Calendar;

public class Logging {
    public Logger logger;
    FileHandler fh;
    String file_name = Robotconfig.teamName + Calendar.getInstance().g().toString() + ".txt";

    public void log(String file_name) throws SecurityException, IOException {
        File f = new File(file_name);
        if(!f.exists()){
            f.createNewFile();
        }

        fh = new FileHandler(file_name, true);
        logger = Logger.getLogger("test");
        logger.addHandler(fh);
        SimpleFormatter formatter = new SimpleFormatter();
        fh.setFormatter(formatter);

    }

}
