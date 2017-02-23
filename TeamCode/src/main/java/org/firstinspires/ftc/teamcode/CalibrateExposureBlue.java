package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

@TeleOp(name = "Calibrate Exposure Blue", group = "Util")
public class CalibrateExposureBlue extends BaseOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    @Override
    public void runOpMode() throws InterruptedException {
        startOpenCV(this);
        waitForStart();
        setButtonsClickable(true);
        while (opModeIsActive()) {
            idle();
        }
        setButtonsClickable(false);
        FtcRobotControllerActivity.mOpenCvCameraView.disableView();
    }

    private void setButtonsClickable(boolean clickable) {
        if (clickable) {
            FtcRobotControllerActivity.setExposureButtonsClickable.obtainMessage().sendToTarget();
        } else {
            FtcRobotControllerActivity.setExposureButtonsUnclickable.obtainMessage().sendToTarget();
        }
    }

    //OpenCV Stuff
    public void onCameraViewStarted(int width, int height) {
        whenCameraViewStarts(width, height, blueHsv, blueContrastRgb);
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return processCameraFrame(inputFrame);
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }
}
