package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

public abstract class BeaconsAutonomous extends CompetitionAutonomous implements CameraBridgeViewBase.CvCameraViewListener2 {

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final int closeToWallDistance = 20;//centimeters
    final int beforePushingButtonDistance = 10;//centimeters

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        startOpenCV(this);
        allInit();
        loadTapeValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        setTeleOpAngle();

        if (shouldShoot()) {
            collectionMotor.setPower(1);
            startLauncher();
            sleep(250);
            moveAwayFromWallAfterCollecting(DEFAULT_SIDEWAYS_SPEED, 15);
            turnAwayFromBeacons(DEFAULT_SPIN_SPEED, 90);
            collectionMotor.setPower(0);
            findVortex();
            shoot();
            fixPosAfterShooting();
        } else {
            moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 10);
        }
        turnToBeacons(DEFAULT_SPIN_SPEED, 45);
        moveAlongBeaconWallDistance(1, 130);
        turnAwayFromBeacons(DEFAULT_SPIN_SPEED, 45);
        goToBeaconWall(0.75, closeToWallDistance);
        fixPosForFindingTape();//Does nothing right now
        findTapeInward();
        alignWithWall();
        pushButton();
        followBeaconWallDistance(1, 97);
        findTapeInward();
        alignWithWall();
        pushButton();
        moveAlongStartWallDistance(-0.75, 10);
        turnToBeacons(DEFAULT_SPIN_SPEED, 45);
        moveAlongBeaconWallDistance(-DEFAULT_FORWARD_SPEED, 175);

        while (opModeIsActive()) {
            idle();
        }
        FtcRobotControllerActivity.mOpenCvCameraView.disableView();
    }

    public abstract boolean shouldShoot();

    public abstract void fixPosAfterShooting() throws InterruptedException;

    public abstract void turnToBeacons(double speed, double angle) throws InterruptedException;

    public abstract void turnAwayFromBeacons(double speed, double angle) throws InterruptedException;

    public void goToBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        moveAlongStartWall(speed);
        while (getRangeDistance() > cmFromWall && opModeIsActive()) {
            sleep(1);
        }
        stopRobot();
    }

    public void goAwayFromBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        moveAlongStartWall(-speed);
        while (getRangeDistance() < cmFromWall && opModeIsActive()) {
            sleep(1);
        }
        stopRobot();
    }

    public void alignWithWall() throws InterruptedException {
//        double angleOffset = determineAngleOffset();
//        if (angleOffset > 2) {
//            spinRightDegrees(0.25, angleOffset);
//        } else if (angleOffset < -2) {
//            spinLeftDegrees(0.25, -angleOffset);
//        }
        goAwayFromBeaconWall(0.37, beforePushingButtonDistance);
        goToBeaconWall(0.37, beforePushingButtonDistance);
        stopRobot();
    }

    public void goForwardAndFollowWall(double speed, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        double totalEncoderTicks = centimeters * TICKS_PER_CM_FORWARD;
        if (Math.abs(speed) > 0.75) {
            double goSlowEncoderTicks = 5 * TICKS_PER_CM_FORWARD;
            double goFastEncoderTicks = (centimeters - 30) * TICKS_PER_CM_FORWARD;
            goForwardAndAdjustToWall(speed / 2, goSlowEncoderTicks);
            goForwardAndAdjustToWall(speed, goFastEncoderTicks);
            goForwardAndAdjustToWall(speed / 2, totalEncoderTicks);
        } else {
            goForwardAndAdjustToWall(speed, totalEncoderTicks);
        }
        stopRobot();
    }

    public void goForwardAndAdjustToWall(double speed, double encoderTicks) {
        boolean adjustedCloser = false;
        boolean adjustedFarther = false;
        boolean wasStraight = false;
        while (getFurthestEncoder() < encoderTicks && opModeIsActive()) {
            if (getRangeDistance() > beforePushingButtonDistance && !adjustedCloser) {
                backLeftMotor.setPower(speed * 0.85);
                backRightMotor.setPower(-speed);
                frontLeftMotor.setPower(speed * 0.85);
                frontRightMotor.setPower(-speed);
                adjustedCloser = true;
                adjustedFarther = false;
                wasStraight = false;
            } else if (getRangeDistance() < beforePushingButtonDistance && !adjustedFarther) {
                backLeftMotor.setPower(speed);
                backRightMotor.setPower(-speed * 0.85);
                frontLeftMotor.setPower(speed);
                frontRightMotor.setPower(-speed * 0.85);
                adjustedFarther = true;
                adjustedCloser = false;
                wasStraight = false;
            } else if (!wasStraight) {
                goForward(speed);
                wasStraight = true;
                adjustedCloser = false;
                adjustedFarther = false;
            }
        }
    }

    //These movements are with respect to the field. Different for red and blue because they mirror each other.
    public abstract void moveAcrossField(double speed);

    public abstract void moveAcrossFieldDistance(double speed, double centimeters) throws InterruptedException;

    public void moveAlongStartWall(double speed) {
        goLeft(speed);
    }

    public void moveAlongStartWallDistance(double speed, double centimeters) throws InterruptedException {
        goLeftDistance(speed, centimeters);
    }

    public abstract void moveAwayFromWallAfterCollecting(double speed, double centimeters) throws InterruptedException;

    public abstract void moveAlongBeaconWall(double speed);

    public abstract void moveAlongBeaconWallDistance(double speed, double centimeters) throws InterruptedException;

    public abstract void followBeaconWallDistance(double speed, double centimeters) throws InterruptedException;

    public abstract void fixPosForFindingTape() throws InterruptedException;

    public void findTapeRight() throws InterruptedException {
        sleep(250);
        goForward(0.1);
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
        }
        int i = 0;
        while (i < 5 && (frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                moveLeftWheelsForward(0.1);
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                moveRightWheelsForward(0.1);
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", backTapeSensor.alpha());
                    updateTelemetry();
                }
            }
            i++;
        }
        stopRobot();
    }

    public void findTapeLeft() throws InterruptedException {
        sleep(250);
        goBackward(0.1);
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
        }
        int i = 0;
        while (i < 5 && (frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                moveLeftWheelsBackward(0.1);
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                moveRightWheelsBackward(0.1);
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", backTapeSensor.alpha());
                    updateTelemetry();
                }
            }
            i++;
        }
        stopRobot();
    }

    public abstract void findTapeInward() throws InterruptedException;

    public abstract void findTapeOutward() throws InterruptedException;

    public abstract void moveCorrectButtonFlap() throws InterruptedException;

    public void pushButton() throws InterruptedException {
        int i = 0;
        do {
            moveCorrectButtonFlap();
            sleep(250);
            resetButtonFlaps();
            sleep(250);
            i++;
        } while (!buttonIsPressed() && i < 3);
    }

    public boolean buttonIsPressed() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        if (Math.abs(leftBlue - rightBlue) < 5 && Math.abs(leftRed - rightRed) < 5) {
            return true;
        }
        return false;
    }

    public void resetButtonFlaps() {
        leftFlapServo.setPosition(leftFlapInitPos);
        rightFlapServo.setPosition(rightFlapInitPos);
    }

    public abstract void setTeleOpAngle();

    public void loadTapeValues() {
        double frontGround = FtcRobotControllerActivity.calibrationSP.getFloat("frontGroundValue", -1000);
        double frontDiff = FtcRobotControllerActivity.calibrationSP.getFloat("frontTapeValue", -1000) - frontGround;
        frontTapeLowThreshold = frontGround + frontDiff * 0.65;
        if (frontTapeLowThreshold < 0) {
            frontTapeLowThreshold = 20;
        }
        double backGround = FtcRobotControllerActivity.calibrationSP.getFloat("backGroundValue", -1000);
        double backDiff = FtcRobotControllerActivity.calibrationSP.getFloat("backTapeValue", -1000) - backGround;
        backTapeLowThreshold = backGround + backDiff * 0.65;
        if (backTapeLowThreshold < 0) {
            backTapeLowThreshold = 20;
        }
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