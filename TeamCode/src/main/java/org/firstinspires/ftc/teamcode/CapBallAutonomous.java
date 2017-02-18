package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public abstract class CapBallAutonomous extends CompetitionAutonomous implements CameraBridgeViewBase.CvCameraViewListener2 {

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

    public abstract Scalar getVortexColorHsv();

    public abstract Scalar getVortexOutlineColorRgb();

    //OpenCV Stuff
    public void onCameraViewStarted(int width, int height) {
        whenCameraViewStarts(width, height, getVortexColorHsv(), getVortexOutlineColorRgb());
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return processCameraFrame(inputFrame);
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }
}
