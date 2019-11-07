package org.firstinspires.ftc.robotcontroller.ultro;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
public class UltroActivity {

}
/**
 * This will never be used!

public class UltroActivity extends FtcRobotControllerActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static final String TAG = "ULTROTAG";
    private JavaCameraView camera;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    camera.enableView();
                    Log.d(TAG, "callback loaded opencv");
                    break;
                default:
                    super.onManagerConnected(status);
                    Log.d(TAG, "callback could not load opencv");


            }
        }
    };
    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return inputFrame.gray();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "On create");
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            camera = findViewById(R.id.java_camera_view);

            Log.d(TAG, "Found camera! " + camera);
            camera.setVisibility(View.VISIBLE);
            camera.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
            camera.setCvCameraViewListener(this);
        }else Log.d(TAG, "Camera doesn't have permission!");
    }

    @Override
    public void onResume() {
        Log.d(TAG, "in on resume");
        super.onResume();

        if(!OpenCVLoader.initDebug()) {
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
            Log.d(TAG, (success) ? "Async Init succeeded" : "Async Init failed");
        }else {
            Log.d(TAG, "OPENCV Libarry found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (camera != null)
            camera.disableView();
    }

    public void onDestroy() {
        super.onDestroy();
        if (camera != null)
            camera.disableView();
    }
}
 */

