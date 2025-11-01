package org.firstinspires.ftc.teamcode.aprilTag;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline
{
    private long nativeApriltagPtr;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private final Mat gray = new Mat();
    private final double tagsize, fx, fy, cx, cy;

    public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
    {
        this.tagsize = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        org.opencv.imgproc.Imgproc.cvtColor(input, gray, org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY);

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                nativeApriltagPtr, gray, tagsize, fx, fy, cx, cy
        );

        for (AprilTagDetection detection : detections)
        {
            drawTagOutline(input, detection);
        }

        return input;
    }

    private void drawTagOutline(Mat input, AprilTagDetection detection)
    {
        Point[] corners = new Point[4];
        for (int i = 0; i < 4; i++) corners[i] = new Point(detection.corners[i].x, detection.corners[i].y);

        org.opencv.imgproc.Imgproc.line(input, corners[0], corners[1], new Scalar(0, 255, 0), 2);
        org.opencv.imgproc.Imgproc.line(input, corners[1], corners[2], new Scalar(0, 255, 0), 2);
        org.opencv.imgproc.Imgproc.line(input, corners[2], corners[3], new Scalar(0, 255, 0), 2);
        org.opencv.imgproc.Imgproc.line(input, corners[3], corners[0], new Scalar(0, 255, 0), 2);

        org.opencv.imgproc.Imgproc.putText(
                input,
                String.format("ID %d", detection.id),
                corners[0],
                org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX,
                0.5,
                new Scalar(255, 0, 0),
                2
        );
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }
}
