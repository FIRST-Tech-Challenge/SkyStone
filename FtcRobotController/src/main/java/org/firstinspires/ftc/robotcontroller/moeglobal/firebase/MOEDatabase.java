package org.firstinspires.ftc.robotcontroller.moeglobal.firebase;

import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;

//    public static void initFirebase() {
//    }
public class MOEDatabase {
    public static DatabaseReference
            status, dataRef, logger, commands, opmodes,
            firecode, codeStatus, codeStatusSync, details, firelog;

    public static void init() {
        final FirebaseDatabase database = FirebaseDatabase.getInstance();
        fillRefs(database);
        handleStatus();
    }

    private static void fillRefs(FirebaseDatabase database) {
        dataRef = database.getReference("driver-station-link");
        status = dataRef.child("status");
        logger = dataRef.child("log");
        opmodes = dataRef.child("opmodes");
        firecode = dataRef.child("firecode");
        commands = dataRef.child("RCCommands");
        codeStatus = dataRef.child("code-status");
        details = codeStatus.child("details");
        codeStatusSync = codeStatus.child("synced");
        firelog = dataRef.child("firelog");
    }


    private static void handleStatus() {
        dataRef.child("status").onDisconnect().setValue(false);
        dataRef.child("status").setValue(true);
        codeStatusSync.onDisconnect().setValue(false);
    }


    public static void sendSynced() {
        codeStatusSync.setValue(true);
    }
}
