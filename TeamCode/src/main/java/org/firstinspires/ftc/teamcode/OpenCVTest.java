package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Open CV Test", group = "Test")
public class OpenCVTest extends BaseOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {

    private boolean mIsColorSelected = false;
    private Mat mRgba;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;
    private ColorBlobDetector mDetector;
    private Mat mSpectrum;
    private Size SPECTRUM_SIZE;
    private Scalar CONTOUR_COLOR;

    double vortexWidth = 0;
    double vortexHeight = 0;
    double vortexX = 0;
    double vortexY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        startOpenCV(this);
        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
        FtcRobotControllerActivity.mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);

        mBlobColorHsv = blueHsv;
        CONTOUR_COLOR = blueContrastRgb;

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

            double maxWidth = 0;
            double theHeight = 0;
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
                if (width > maxWidth) {
                    maxWidth = width;
                    theHeight = height;
                    theX = (left + right) / 2;
                    theY = (top + bottom) / 2;
                }
            }
            vortexWidth = maxWidth / mRgba.width();
            vortexHeight = theHeight / mRgba.height();
            vortexX = theX / mRgba.width();
            vortexY = theY / mRgba.height();
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
}