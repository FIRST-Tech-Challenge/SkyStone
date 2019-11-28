package org.firstinspires.ftc.robotlib.debug;

import com.google.gson.JsonObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

import fi.iki.elonen.NanoHTTPD;

public class Webserver extends NanoHTTPD {
    private DebugServer debugServer;

    Webserver(DebugServer debugServer) throws IOException {
        super(5550);
        this.debugServer = debugServer;
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();

        // API
        if (uri.equals("/api/stats")) {
            JsonObject response = new JsonObject();
            response.addProperty("stream", debugServer.getStreamJPEG64());
            return newFixedLengthResponse(response.getAsString());
        }

        // Static Files
        try {
            if (uri.equals("/") || uri.isEmpty()) return newFixedLengthResponse(readFile("client/index"));
            if (uri.startsWith("/assets/")) return newFixedLengthResponse(readFile("client" + uri));
        } catch (IOException e) {
            return newFixedLengthResponse(e.getMessage());
        }

        return newFixedLengthResponse(Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "NOT FOUND");
    }

    private String readFile(String path) throws IOException {
        int identifier = debugServer.hardwareMap.appContext.getResources().getIdentifier(path, "raw", debugServer.hardwareMap.appContext.getPackageName());
        InputStream inputStream = debugServer.hardwareMap.appContext.getResources().openRawResource(identifier);
        ByteArrayOutputStream res = new ByteArrayOutputStream();
        byte[] buffer = new byte[1024];
        int length;
        while ((length = inputStream.read(buffer)) != -1) {
            res.write(buffer, 0, length);
        }
        return res.toString("UTF-8");
    }
}