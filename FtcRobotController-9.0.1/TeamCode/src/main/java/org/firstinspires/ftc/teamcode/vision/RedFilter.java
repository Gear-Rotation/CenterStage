package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RedFilter extends OpenCvPipeline {
    private List<Mat> channels = new ArrayList<>();
    public Vector2d offset = new Vector2d(50,50);
    public Vector2d offset1 = new Vector2d(0,0);


    @Override
    public Mat processFrame(Mat input) {
        channels = new ArrayList<>();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
//        Imgproc.GaussianBlur(input, input, new Size(5,5), 0);
//        Core.split(input, channels);
//
//        if(channels.size() > 0) {
//            Imgproc.threshold(channels.get(2), input, 134, 255, Imgproc.THRESH_BINARY);
//        }
//
//        channels.get(0).release();
//        channels.get(1).release();
//        channels.get(2).release();

        Imgproc.rectangle(
                input,
                new Point(0 + offset.getX(), 0 + offset.getY()),
                new Point(40 + offset.getX(), 40 + offset.getY()),
                new Scalar(255, 0, 0),
                1,
                0);

        Imgproc.rectangle(
                input,
                new Point(0 + offset1.getX(), 0 + offset1.getY()),
                new Point(40 + offset1.getX(), 40 + offset1.getY()),
                new Scalar(0, 255, 0),
                1,
                0);

        return input;
    }
}
