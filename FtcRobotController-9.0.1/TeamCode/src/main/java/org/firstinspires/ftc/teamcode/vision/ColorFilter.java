package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class ColorFilter extends OpenCvPipeline {
    public abstract void proccessFrame(Mat input, Mat mask, double threshold);
}
