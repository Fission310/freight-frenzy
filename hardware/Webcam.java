package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class Webcam extends Mechanism{
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private BarcodeDetector detector;

    public Webcam(LinearOpMode opMode){
        this.opMode = opMode;
    }


    @Override
    public void init(HardwareMap hwMap) {

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

        webcamName = hwMap.get(WebcamName.class, "Webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280, 720);
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        detector = new BarcodeDetector(opMode.telemetry);



    }

    public void setPipeline(){
        camera.setPipeline(detector);
    }

    public BarcodeDetector.Location location(){
        return detector.getLocation();
    }

    public void stopStreaming() { camera.stopStreaming(); }


}
