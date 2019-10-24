package org.firstinspires.ftc.robotcontroller.moeglobal.firebase;

import android.os.Environment;
import com.google.firebase.storage.FileDownloadTask;
import com.google.firebase.storage.FirebaseStorage;
import com.google.firebase.storage.StorageReference;
import org.firstinspires.ftc.robotcontroller.moeglobal.FileUtils;

import java.io.File;

public class MOEStorage {
    public static StorageReference storageRef;

    public static void init() {
        FirebaseStorage storage = FirebaseStorage.getInstance();
        storageRef = storage.getReference();

    }


    public static void clearFiles() {
        File file = new File(
                Environment.getExternalStorageDirectory().getAbsolutePath() + "/firecode"
        );
        if (!file.exists()) {
            file.mkdirs();
            return;
        }
        for (File listFile : file.listFiles()) {
            listFile.delete();
        }
    }

    public static FileDownloadTask downloadDexFromUUID(String value) {
        String pathString = "pushcode/" + value;
        StorageReference onlineRef = storageRef.child(pathString);
        File path = FileUtils.getFireCodeFileFromUUID(value);

        return onlineRef.getFile(path);

    }

}
