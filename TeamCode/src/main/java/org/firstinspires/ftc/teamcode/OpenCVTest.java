package org.firstinspires.ftc.teamcode;

import android.view.View;

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

    private Scalar redHsv = new Scalar(0, 255, 255);
    private Scalar blueHsv = new Scalar(170, 255, 255);
    private Scalar redContrastRgb = new Scalar(0, 0, 255, 255);
    private Scalar blueContrastRgb = new Scalar(255, 0, 0, 255);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcRobotControllerActivity.mOpenCvCameraView.setVisibility(View.VISIBLE);
        FtcRobotControllerActivity.mOpenCvCameraView.setCvCameraViewListener(this);
        FtcRobotControllerActivity.textDataLog.setVisibility(View.GONE);
        allInit();
        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);

        mBlobColorHsv = redHsv;
        CONTOUR_COLOR = redContrastRgb;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

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
            double maxHeight = 0;
            for (MatOfPoint contour : mDetector.getContours()) {
                double left = contour.toArray()[0].x;
                double right = contour.toArray()[0].x;
                double top = contour.toArray()[0].y;
                double bottom = contour.toArray()[0].y;
                for (Point p : contour.toArray()) {
                    left = p.x < left ? p.x : left;
                    right = p.x > right ? p.x : right;
                    top = p.y < top ? p.y : top;
                    bottom = p.y > bottom ? p.y : bottom;
                }
                double width = right - left;
                double height = bottom - top;
                maxWidth = width > maxWidth ? width : maxWidth;
                maxHeight = height > maxHeight ? height : maxHeight;
            }
            if (telemetry != null)
                logData("width", maxWidth + ", height: " + maxHeight);
        }

        return mRgba;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

    static class ColorBlobDetector {
        // Lower and Upper bounds for range checking in HSV color space
        private Scalar mLowerBound = new Scalar(0);
        private Scalar mUpperBound = new Scalar(0);
        // Minimum contour area in percent for contours filtering
        private static double mMinContourArea = 0.1;
        // Color radius for range checking in HSV color space
        private Scalar mColorRadius = new Scalar(30, 100, 100, 0);
        private Mat mSpectrum = new Mat();
        private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

        // Cache
        Mat mPyrDownMat = new Mat();
        Mat mHsvMat = new Mat();
        Mat mMask = new Mat();
        Mat mMask1 = new Mat();
        Mat mMask2 = new Mat();
        Mat mDilatedMask = new Mat();
        Mat mHierarchy = new Mat();

        public void setColorRadius(Scalar radius) {
            mColorRadius = radius;
        }

        public void setHsvColor(Scalar hsvColor) {
            double minH = hsvColor.val[0] - mColorRadius.val[0];
            double maxH = hsvColor.val[0] + mColorRadius.val[0];

            mLowerBound.val[0] = minH;
            mUpperBound.val[0] = maxH;

            mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
            mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

            mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
            mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

            mLowerBound.val[3] = 0;
            mUpperBound.val[3] = 255;

            Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

            for (int j = 0; j < maxH - minH; j++) {
                int minHPlusJ = (int) (minH + j);
                if (minHPlusJ < 0) {
                    minHPlusJ += 256;
                }
                byte[] tmp = {(byte) minHPlusJ, (byte) 255, (byte) 255};
                spectrumHsv.put(0, j, tmp);
            }

            Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
        }

        public Mat getSpectrum() {
            return mSpectrum;
        }

        public void setMinContourArea(double area) {
            mMinContourArea = area;
        }

        public void process(Mat rgbaImage) {
            Imgproc.pyrDown(rgbaImage, mPyrDownMat);
            Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

            Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

            if (mLowerBound.val[0] < 0) {
                Scalar lowBound = new Scalar(mLowerBound.val[0] + 256, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
                Scalar minBound = new Scalar(0, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
                Scalar maxBound = new Scalar(255, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
                Core.inRange(mHsvMat, minBound, mUpperBound, mMask1);
                Core.inRange(mHsvMat, lowBound, maxBound, mMask2);
                Core.bitwise_or(mMask2, mMask1, mMask);
            } else if (mUpperBound.val[0] > 255) {
                Scalar highBound = new Scalar(mUpperBound.val[0] - 256, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
                Scalar minBound = new Scalar(0, mLowerBound.val[1], mLowerBound.val[2], mLowerBound.val[3]);
                Scalar maxBound = new Scalar(255, mUpperBound.val[1], mUpperBound.val[2], mUpperBound.val[3]);
                Core.inRange(mHsvMat, minBound, highBound, mMask1);
                Core.inRange(mHsvMat, mLowerBound, maxBound, mMask2);
                Core.bitwise_or(mMask2, mMask1, mMask);
            } else {
                Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
            }
            Imgproc.dilate(mMask, mDilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find max contour area
            double maxArea = 0;
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > maxArea)
                    maxArea = area;
            }

            // Filter contours by area and resize to fit the original image size
            mContours.clear();
            each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint contour = each.next();
                if (Imgproc.contourArea(contour) > mMinContourArea * maxArea) {
                    Core.multiply(contour, new Scalar(4, 4), contour);
                    mContours.add(contour);
                }
            }
        }

        public List<MatOfPoint> getContours() {
            return mContours;
        }
    }
}