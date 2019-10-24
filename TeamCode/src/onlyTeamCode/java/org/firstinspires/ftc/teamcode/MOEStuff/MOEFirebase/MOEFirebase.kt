package org.firstinspires.ftc.teamcode.MOEStuff.MOEFirebase;

import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseReference

interface MOEFirebase {
    //    var localConfigSnapshot: DataSnapshot
    fun getCustomRef(ref: DatabaseReference): DatabaseReference? {
        return null
    }

    fun onConfigChanged(dataSnapshot: DataSnapshot) {

    }
}