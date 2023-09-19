package org.firstinspires.ftc.teamcode.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat submat;

    @Override
    public void init(Mat firstFrame)
    {
        submat = firstFrame.submat(0,50,0,50);
    }
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
