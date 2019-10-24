package org.firstinspires.ftc.teamcode.MOEStuff.MOEFirebase

import com.google.firebase.database.ChildEventListener
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.ValueEventListener

abstract class MOEEventListener : ValueEventListener {

    abstract override fun onDataChange(snapshot: DataSnapshot)
    override fun onCancelled(p0: DatabaseError) {
    }
}