package org.firstinspires.ftc.teamcode.teamcode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.teamcode.internal.OptimizedOpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import java.util.concurrent.atomic.AtomicReference;

import java.util.ArrayList;

@Config
public class Pipeline extends OptimizedOpenCVPipeline {
    public static int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public static float DECIMATION_HIGH = 3;
    public static float DECIMATION_LOW = 2;
    public static float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    int numFramesWithoutDetection = 0;
    private long nativeApriltagPtr;
    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7, 197, 235, 255);
    Scalar red = new Scalar(255, 0, 0, 255);
    Scalar green = new Scalar(0, 255, 0, 255);
    Scalar white = new Scalar(255, 255, 255, 255);

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double tagsize = 0.0508;
    public static double tagsizeX = tagsize;
    public static double tagsizeY = tagsize;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(800, 448, Bitmap.Config.RGB_565));

    private Mode mode = Mode.Spike;
    private final Team team;
    private final Distance distance;

    public static int spikeSize = 25;

    public static int leftSpikeTopLeftX = 90;
    public static int leftSpikeTopLeftY = 200;
    public static int centerSpikeTopLeftX = 315;
    public static int centerSpikeTopLeftY = 200;
    public static int rightSpikeTopLeftX = 590;
    public static int rightSpikeTopLeftY = 200;
    public static int redOffset = 115;

    public SpikePos spikePos = null;

    private Mat leftSubmat;
    private Mat centerSubmat;
    private Mat rightSubmat;

    @Override
    public void init(Mat mat) {
        int xOffset = team == Team.Red ^ distance == Distance.Far ? redOffset : 0;
        leftSubmat = mat.submat(leftSpikeTopLeftY, leftSpikeTopLeftY + spikeSize, leftSpikeTopLeftX + xOffset, leftSpikeTopLeftX + xOffset + spikeSize);
        centerSubmat = mat.submat(centerSpikeTopLeftY, centerSpikeTopLeftY + spikeSize, centerSpikeTopLeftX + xOffset, centerSpikeTopLeftX + xOffset + spikeSize);
        rightSubmat = mat.submat(rightSpikeTopLeftY, rightSpikeTopLeftY + spikeSize, rightSpikeTopLeftX + xOffset, rightSpikeTopLeftX + xOffset + spikeSize);
        lastFrame.set(Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565));
    }

    public Pipeline(Team team, Distance distance) {
        this.team = team;
        this.distance = distance;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    protected void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat output;
        switch (mode) {
            case April:
                output = processApril(input);
                break;
            case Spike:
                output = processSpike(input);
                break;
            case Idle:
                output = input;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + mode);
        }
        Bitmap b = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, b);
        lastFrame.set(b);
        return output;
    }

    private Mat processSpike(Mat input) {
        int xOffset = team == Team.Red ^ distance == Distance.Far ? redOffset : 0;
        Scalar leftColor = Core.mean(leftSubmat);
        Scalar centerColor = Core.mean(centerSubmat);
        Scalar rightColor = Core.mean(rightSubmat);
        Imgproc.rectangle(
                input,
                new Point(leftSpikeTopLeftX + xOffset, leftSpikeTopLeftY),
                new Point(leftSpikeTopLeftX + xOffset + spikeSize, leftSpikeTopLeftY + spikeSize),
                new Scalar(0, 0, 0),
                2
        );
        Imgproc.rectangle(
                input,
                new Point(centerSpikeTopLeftX + xOffset, centerSpikeTopLeftY),
                new Point(centerSpikeTopLeftX + xOffset + spikeSize, centerSpikeTopLeftY + spikeSize),
                new Scalar(0, 0, 0),
                2
        );
        Imgproc.rectangle(
                input,
                new Point(rightSpikeTopLeftX + xOffset, rightSpikeTopLeftY),
                new Point(rightSpikeTopLeftX + xOffset + spikeSize, rightSpikeTopLeftY + spikeSize),
                new Scalar(0, 0, 0),
                2
        );
        // magic color shit
        // find the one that is most right and least wrong
        double leftTarget = leftColor.val[team.colorOffset] - leftColor.val[(team.colorOffset + 1) % 3] - leftColor.val[(team.colorOffset + 2) % 3];
        double rightTarget = rightColor.val[team.colorOffset] - rightColor.val[(team.colorOffset + 1) % 3] - rightColor.val[(team.colorOffset + 2) % 3];
        double centerTarget =  centerColor.val[team.colorOffset] - centerColor.val[(team.colorOffset + 1) % 3] - centerColor.val[(team.colorOffset + 2) % 3];
        if (leftTarget > rightTarget && leftTarget > centerTarget) {
            spikePos = SpikePos.Left;
        } else if (rightTarget > centerTarget) {
            spikePos = SpikePos.Right;
        } else {
            spikePos = SpikePos.Center;
        }
        return input;
    }

    private Mat processApril(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image.
        for (AprilTagDetection detection : detections) {
            Pose pose = aprilTagPoseToOpenCvPose(detection.pose);
            //Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
            drawAxisMarker(input, tagsizeY / 2.0, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }
        return input;
    }

    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            if (ret != null) {
                if (ret.isEmpty()) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;
                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        setDecimation(DECIMATION_HIGH);
                    }
                }
            }
            return ret;
        }
    }

    void constructMatrix() {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf          the RGB buffer on which to draw the marker
     * @param length       the length of each of the marker 'poles'
     * @param rvec         the rotation vector of the detection
     * @param tvec         the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, Mat rvec, Mat tvec, Mat cameraMatrix) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(length, 0, 0),
                new Point3(0, length, 0),
                new Point3(0, 0, -length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, 6);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, 6);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, 6);

        Imgproc.circle(buf, projectedPoints[0], 6, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, tagHeight / 2, 0),
                new Point3(tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, -tagHeight / 2, 0),
                new Point3(-tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, tagHeight / 2, -length),
                new Point3(tagWidth / 2, -tagHeight / 2, -length),
                new Point3(-tagWidth / 2, -tagHeight / 2, -length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose) {
        Pose pose = new Pose();
        pose.tvec.put(0, 0, aprilTagPose.x);
        pose.tvec.put(1, 0, aprilTagPose.y);
        pose.tvec.put(2, 0, aprilTagPose.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.put(i, j, aprilTagPose.R.get(i, j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        return pose;
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points       the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX     the original width of the tag
     * @param tagsizeY     the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX, double tagsizeY) {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX / 2, -tagsizeY / 2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX / 2, -tagsizeY / 2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    static class Pose {
        Mat rvec;
        Mat tvec;

        public Pose() {
            rvec = new Mat(3, 1, CvType.CV_32F);
            tvec = new Mat(3, 1, CvType.CV_32F);
        }

        public Pose(Mat rvec, Mat tvec) {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

    public enum Mode {
        Spike, Idle, April
    }

    public enum Distance {
        Close, Far
    }

    public enum Team {
        Red(0), Blue(2);
        public final int colorOffset;

        private Team(int colorOffset) {
            this.colorOffset = colorOffset;
        }
    }

    public enum SpikePos {
        Left, Right, Center;

        public int targetTag() {
            switch (this) {
                case Left:
                    return 1;
                case Right:
                    return 2;
                case Center:
                    return 3;
            }
            return 0;
        }
    }

    public void finishSpike() {
        mode = Mode.Idle;
    }

    public void startApril() {
        mode = Mode.April;
    }
}