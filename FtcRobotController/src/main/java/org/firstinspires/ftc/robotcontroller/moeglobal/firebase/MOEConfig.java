package org.firstinspires.ftc.robotcontroller.moeglobal.firebase;

import androidx.annotation.NonNull;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.ValueEventListener;

import static org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase.dataRef;

public class MOEConfig {

    public static DatabaseReference config;
    public static DataSnapshot snapshot;

    public static void init() {
        config = dataRef.child("config");
        addDataListener();
    }

    private static void addDataListener() {
        config.addValueEventListener(new ValueEventListener() {
            @Override
            public void onDataChange(@NonNull DataSnapshot dataSnapshot) {
                snapshot = dataSnapshot;
            }

            @Override
            public void onCancelled(@NonNull DatabaseError databaseError) {

            }
        });
    }
}
