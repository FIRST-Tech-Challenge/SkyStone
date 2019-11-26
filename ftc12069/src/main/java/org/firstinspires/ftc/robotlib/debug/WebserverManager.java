package org.firstinspires.ftc.robotlib.debug;

import java.io.IOException;

public class WebserverManager {
    private Webserver webserver;

    public WebserverManager() throws IOException {
        webserver = new Webserver();
    }
}
