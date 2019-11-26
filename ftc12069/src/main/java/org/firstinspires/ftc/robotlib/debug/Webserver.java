package org.firstinspires.ftc.robotlib.debug;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Scanner;

import fi.iki.elonen.NanoHTTPD;

public class Webserver extends NanoHTTPD {
    public Webserver() throws IOException {
        super(5550);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();
        /*if (uri.equals("/api/stats")) {

        }*/

        try {
            return newFixedLengthResponse(readFile("client/index.html"));
        } catch (IOException e) {
            return newFixedLengthResponse(e.getMessage());
        }
    }

    private String readFile(String path) throws IOException {
        // fileInputStream = new FileInputStream(path);
        InputStream inputStream = getClass().getResourceAsStream(path);
        int content;
        StringBuilder result = new StringBuilder();
        if (inputStream != null) {
            while ((content = inputStream.read()) != -1) {
                result.append((char) content);
            }
        }
        return result.toString();
    }
}