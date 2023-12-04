package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class FrameGrabberBlue2 {
    private Mat displayMat = new Mat();
    private Mat workingMat = new Mat();
    private Mat redMask = new Mat();

    public BlueFilterFar blueFilterFar;
    private Telemetry telemetry;
    private Robot robot;
    private OpenCvCamera camera;

    public FrameGrabberBlue2(OpMode opMode, Robot robot) {
        telemetry = opMode.telemetry;
        this.robot = robot;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        blueFilterFar = new BlueFilterFar();
        camera.setPipeline(blueFilterFar);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }

    public void closeCamera() {
        camera.closeCameraDevice();
    }
}
