package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public abstract class CapBallAutonomous extends CompetitionAutonomous implements CameraBridgeViewBase.CvCameraViewListener2 {

    private boolean seesVortex = false;
    private final double minVortexWidth = 0.2;
    private double vortexWidth = 0;
    private double vortexHeight = 0;
    private double vortexX = 0;
    private double vortexY = 0;

    @Override
    public int numParticles() {
        return 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        startOpenCV(this);
        allInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        goBackwardDistance(DEFAULT_FORWARD_SPEED, 60);
        findVortex();
        shoot();
        sleep(13000);
        spinRightDegrees(DEFAULT_SPIN_SPEED, 180);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 115);
        sleep(2000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 12);


        while (opModeIsActive()) {
            idle();
        }
        FtcRobotControllerActivity.mOpenCvCameraView.disableView();
    }

    private void findVortex() {
        while (vortexX < 0.5 && seesVortex) {
            spinRight(0.25);
        }
        while (vortexX > 0.5 && seesVortex) {
            spinLeft(0.25);
        }
        while (vortexWidth < vortexTargetWidthLong && seesVortex) {
            goBackward(0.25);
        }
        while (vortexWidth > vortexTargetWidthLong && seesVortex) {
            goForward(0.25);
        }
        stopRobot();
    }

    public abstract Scalar getVortexColorHsv();

    public abstract Scalar getVortexOutlineColorRgb();

    //OpenCV Stuff
    private boolean mIsColorSelected = false;
    private Mat mRgba;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;
    private ColorBlobDetector mDetector;
    private Mat mSpectrum;
    private Size SPECTRUM_SIZE;
    private Scalar CONTOUR_COLOR;

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);

        mBlobColorHsv = getVortexColorHsv();
        CONTOUR_COLOR = getVortexOutlineColorRgb();

        mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);

        mDetector.setHsvColor(mBlobColorHsv);

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();

        if (mIsColorSelected) {
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            if (telemetry != null)
                logData("Contours count", contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);

            seesVortex = false;
            double maxHeight = 0;
            double theWidth = 0;
            double theX = 0;
            double theY = 0;
            for (MatOfPoint contour : mDetector.getContours()) {
                Point[] points = contour.toArray();
                double left = points[0].x;
                double right = points[0].x;
                double top = points[0].y;
                double bottom = points[0].y;
                for (Point p : points) {
                    left = p.x < left ? p.x : left;
                    right = p.x > right ? p.x : right;
                    top = p.y < top ? p.y : top;
                    bottom = p.y > bottom ? p.y : bottom;
                }
                double width = right - left;
                double height = bottom - top;
                if (height > minVortexWidth && height > maxHeight) {
                    seesVortex = true;
                    maxHeight = height;
                    theWidth = width;
                    theX = (left + right) / 2;
                    theY = (top + bottom) / 2;
                }
            }
            vortexWidth = maxHeight / mRgba.height(); // Reversed because screen orientation is sideways, but phone is vertical
            vortexHeight = theWidth / mRgba.width();
            vortexX = theY / mRgba.height();
            vortexY = theX / mRgba.width();
            if (telemetry != null) {
                logData("width", vortexWidth);
                logData("height", vortexHeight);
                logData("x", vortexX);
                logData("y", vortexY);
            }

            if (telemetry != null)
                updateTelemetry();
        }
        return mRgba;
    }

    private Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
}
