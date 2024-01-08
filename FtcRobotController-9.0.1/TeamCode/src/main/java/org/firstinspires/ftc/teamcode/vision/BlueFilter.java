package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class BlueFilter extends OpenCvPipeline {
    private List<Mat> channels = new ArrayList<>();

    //center
    public Vector2d offset = new Vector2d(238,302);
    //right
    public Vector2d offset1 = new Vector2d(580,278);
    private Mat workingMat = new Mat();
    private Mat maskMat = new Mat();

public double threshold = 134;
    public double offsetAverage = 0;
    public double offset1Average = 0;

    public enum Status {
        NONE,
        PRESENT
    }

    public enum State {
        LEFT,
        RIGHT,
        CENTER,
        NOT_FOUND
    }
    public Status leftStatus = Status.NONE;
    public Status rightStatus = Status.NONE;

    public State position = State.NOT_FOUND;


    @Override
    public Mat processFrame(Mat input) {
        channels = new ArrayList<>();
        input.copyTo(workingMat);

        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.GaussianBlur(workingMat, workingMat, new Size(5,5), 0);
        Core.split(workingMat, channels);

        if(channels.size() > 0) {
            Imgproc.threshold(channels.get(2), workingMat, threshold, 255, Imgproc.THRESH_BINARY);
        }

        channels.get(0).release();
        channels.get(1).release();
        channels.get(2).release();

        Imgproc.rectangle(
                input,
                new Point(0 + offset.getX(), 0 + offset.getY()),
                new Point(100 + offset.getX(), 100 + offset.getY()),
                new Scalar(255, 0, 0),
                2,
                0);

        Imgproc.rectangle(
                input,
                new Point(0 + offset1.getX(), 0 + offset1.getY()),
                new Point(100 + offset1.getX(), 100 + offset1.getY()),
                new Scalar(0, 255, 0),
                2,
                0);

        maskMat = workingMat.submat(new Rect((int) offset.getX(), (int) offset.getY(), 100, 100));
        offsetAverage = Core.mean(maskMat).val[0];

        maskMat = workingMat.submat(new Rect((int) offset1.getX(), (int) offset1.getY(), 100, 100));
        offset1Average = Core.mean(maskMat).val[0];

        if(offsetAverage >= 150) {
            leftStatus = Status.PRESENT;
        } else {
            leftStatus = Status.NONE;
        }

        if(offset1Average >= 150) {
            rightStatus = Status.PRESENT;
        } else {
            rightStatus = Status.NONE;
        }

        if(rightStatus == Status.PRESENT) {
            position = State.RIGHT;

        } else if(leftStatus == Status.PRESENT) {
            position = State.CENTER;

        } else if(leftStatus == Status.NONE && rightStatus == Status.NONE) {
            position = State.LEFT;

        }

//        workingMat.release();

        return workingMat;
    }
}
