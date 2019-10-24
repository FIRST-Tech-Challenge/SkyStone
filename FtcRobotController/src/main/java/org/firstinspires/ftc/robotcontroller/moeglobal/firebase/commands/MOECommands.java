package org.firstinspires.ftc.robotcontroller.moeglobal.firebase.commands;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.google.firebase.database.ChildEventListener;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.DatabaseReference.CompletionListener;
import com.qualcomm.robotcore.exception.RobotCoreException;
import org.firstinspires.ftc.robotcontroller.moeglobal.ActivityReferenceHolder;
import org.firstinspires.ftc.robotcontroller.moeglobal.firebase.MOEDatabase;

public class MOECommands {
    public static void init() {
        MOEDatabase.commands.setValue(null, new CompletionListener() {
            @Override
            public void onComplete(@Nullable final DatabaseError databaseError, @NonNull DatabaseReference databaseReference) {
                databaseReference.addChildEventListener(new ChildEventListener() {
                    @Override
                    public void onChildAdded(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {

                        handleCommandAdded(dataSnapshot.getValue(FireCommand.class));
                    }

                    @Override
                    public void onChildChanged(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {

                    }

                    @Override
                    public void onChildRemoved(@NonNull DataSnapshot dataSnapshot) {

                    }

                    @Override
                    public void onChildMoved(@NonNull DataSnapshot dataSnapshot, @Nullable String s) {

                    }

                    @Override
                    public void onCancelled(@NonNull DatabaseError databaseError) {

                    }
                });
            }
        });
    }

    private static void handleCommandAdded(FireCommand value) {
        try {
            ActivityReferenceHolder.activityRef.get().eventLoop.processCommand(value.toRealCommand());
        } catch (InterruptedException | RobotCoreException e) {
            e.printStackTrace();
        }
    }


}
