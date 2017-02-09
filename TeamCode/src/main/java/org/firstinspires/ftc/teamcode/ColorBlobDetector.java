package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ColorBlobDetector {
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
