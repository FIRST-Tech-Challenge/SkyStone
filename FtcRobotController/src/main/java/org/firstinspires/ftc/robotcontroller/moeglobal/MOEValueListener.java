package org.firstinspires.ftc.robotcontroller.moeglobal;

import androidx.annotation.NonNull;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.ValueEventListener;


public abstract class MOEValueListener implements ValueEventListener {
    @Override
    public void onCancelled(@NonNull DatabaseError databaseError) {

    }
}

