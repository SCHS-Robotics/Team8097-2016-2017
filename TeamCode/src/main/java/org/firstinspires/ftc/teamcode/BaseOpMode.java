package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_FORWARD_SPEED = 0.75;
    public final static double DEFAULT_DIAGONAL_SPEED = 0.85;
    public final static double DEFAULT_SIDEWAYS_SPEED = 0.75;
    public final static double DEFAULT_SPIN_SPEED = 0.75;

    public final static double MIN_SPEED = 0.1;

    public final static double TICKS_PER_CM_FORWARD = 53.6 / 1.5; //Divided by 1.5 because we switched from 60s to 40s
    public final static double TICKS_PER_CM_SIDEWAYS = 72.0 / 1.5;
    public final static double TICKS_PER_CM_DIAGONAL = 88.3 / 1.5;
    public final static double TICKS_PER_DEGREE = 27.5 / 1.5;

    public final static double SERVO_POS_PER_DEGREE = 1.0 / 151.0;

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftLaunchMotor;
    DcMotor rightLaunchMotor;
    DcMotor collectionMotor;
    HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();
    int wheelEncoderPpr = 1680;
    int launcherEncoderPpr = 112;
    int wheelMaxRpm = 100;//theoretical 110
    int launcherMaxRpm = 1450;//theoretical 1650
    Servo rightFlapServo;
    Servo leftFlapServo;
    Servo rangeServo;
    Servo launcherServo;
    Servo leftLiftServo;
    Servo rightLiftServo;
    ColorSensor rightColorSensor;
    ColorSensor leftColorSensor;
    ColorSensor frontTapeSensor;
    ColorSensor backTapeSensor;
    I2cAddr leftColorI2c = I2cAddr.create8bit(0x3c);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr frontTapeI2c = I2cAddr.create8bit(0x5c);
    I2cAddr backTapeI2c = I2cAddr.create8bit(0x6c);
    RangeSensor rangeSensor;
    //    RangeSensor rightRangeSensor;
    I2cAddr rangeI2c = I2cAddr.create8bit(0x28);
//    I2cAddr rightRangeI2c = I2cAddr.create8bit(0x38);

    final double leftFlapInitPos = 0.710;
    final double rightFlapInitPos = 0.324;
    final double leftFlapEndPos = 0.512;
    final double rightFlapEndPos = 0.502;
    final double rangeServoInitPos = 0.518;
    final double launcherServoAutoPos = 0.840;
    final double launcherServoShortPos = 0.332;
    final double launcherServoFarPos = launcherServoAutoPos;
    final double launcherServoInitPos = launcherServoAutoPos;
    final double leftLiftInitPos = 0.636;
    final double rightLiftInitPos = 0.210;
    final double leftLiftEndPos = 0.220;
    final double rightLiftEndPos = 0.600;

    final int launchFarServoWaitTime = 23;//milliseconds
    final int launchShortServoWaitTime = 13;//milliseconds

    final double[][] vortexValues = {//FIRST COLUMN MUST BE INCREASING
            {0.328, 0.840},
            {0.370, 0.830},
            {0.396, 0.758},
            {0.417, 0.660},
            {0.438, 0.538},
            {0.474, 0.408},
            {0.505, 0.328},
            {0.547, 0.240}};

    final double vortexTargetWidthLong = 0.34;
    final double vortexTargetWidthShort = 0.479;
    final double minVortexWidth = 0.2;
    final double vortexTargetX = 0.52;
    final Scalar redHsv = new Scalar(243, 166, 166);
    final Scalar blueHsv = new Scalar(148, 166, 142);
    final Scalar redContrastRgb = new Scalar(0, 0, 255, 255);
    final Scalar blueContrastRgb = new Scalar(255, 0, 0, 255);

    private HashMap<String, Object> telemetryData = new HashMap<String, Object>();

    public void logData(String label, Object value) {
        telemetry.addData(label, value);
        telemetryData.put(label, value);
    }

    public void updateTelemetry() {
        updateTelemetry(telemetry);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        if (!FtcRobotControllerActivity.logData.hasMessages(0)) {
            StringBuilder data = new StringBuilder();
            for (String key : telemetryData.keySet()) {
                data.append(key).append(": ").append(telemetryData.get(key)).append("\n");
            }
            FtcRobotControllerActivity.logData.obtainMessage(0, data.toString()).sendToTarget();
            telemetryData.clear();
        }
    }

    public void startLauncher() {
        leftLaunchMotor.setPower(-0.84);
        rightLaunchMotor.setPower(0.84);
    }

    public void stopLauncher() {
        leftLaunchMotor.setPower(0);
        rightLaunchMotor.setPower(0);
    }

    public void liftToLaunch() throws InterruptedException {
        leftLiftServo.setPosition(leftLiftEndPos);
        sleep(launcherServoDelay());
        rightLiftServo.setPosition(rightLiftEndPos);
    }

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void spinRight(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    public void spinLeft(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
    }

    public void goForward(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
    }

    public void goBackward(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
    }

    public void goLeft(double speed) {
        backLeftMotor.setPower(speed * 1);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed * 1);
    }

    public void goRight(double speed) {
        backLeftMotor.setPower(-speed * 1);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed * 1);
    }

    public void goDiagonalForwardRight(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(0);
    }

    public void goDiagonalForwardLeft(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-speed);
    }

    public void goDiagonalBackwardRight(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(speed);
    }

    public void goDiagonalBackwardLeft(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(0);
    }

    public void resetWheelEncoders() {
        resetEncoders(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void resetEncoders(DcMotor... motors) {
        for (DcMotor motor : motors) {
            while (motor.getCurrentPosition() != 0)
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderStartPos.put(motor, 0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getCurrentRpm(int encoderPpr, DcMotor motor, int waitTime) {
        return ((double) (Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
    }

    public void initWheels() {
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        backRightMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        frontLeftMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        frontRightMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        resetWheelEncoders();
    }

    public void initLauncher() {
        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        leftLaunchMotor.setMaxSpeed((int) ((launcherMaxRpm * launcherEncoderPpr) / 60.0));
        rightLaunchMotor.setMaxSpeed((int) ((launcherMaxRpm * launcherEncoderPpr) / 60.0));
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetEncoders(leftLaunchMotor, rightLaunchMotor);

        launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(launcherServoInitPos);
        leftLiftServo = hardwareMap.servo.get("leftLift");
        rightLiftServo = hardwareMap.servo.get("rightLift");
        leftLiftServo.setPosition(leftLiftInitPos);
        rightLiftServo.setPosition(rightLiftInitPos);
    }

    public void initCollection() {
        collectionMotor = hardwareMap.dcMotor.get("collect");
    }

    public void initRange() {
        //        I2cDevice rightRangeDevice = hardwareMap.i2cDevice.get("rightRange");
        I2cDevice rangeDevice = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch rightRangeReader = new I2cDeviceSynchImpl(rightRangeDevice, rightRangeI2c, false);
        I2cDeviceSynch rangeReader = new I2cDeviceSynchImpl(rangeDevice, rangeI2c, false);
//        rightRangeSensor = new RangeSensor(rightRangeReader);
        rangeSensor = new RangeSensor(rangeReader);

        rangeServo = hardwareMap.servo.get("rangeServo");
        rangeServo.setPosition(rangeServoInitPos);
    }

    public void initTape() {
        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        frontTapeSensor.setI2cAddress(frontTapeI2c);
        backTapeSensor.setI2cAddress(backTapeI2c);
        frontTapeSensor.enableLed(true);
        backTapeSensor.enableLed(true);
    }

    public void initButtons() {
        rightColorSensor = hardwareMap.colorSensor.get("rightColor");
        leftColorSensor = hardwareMap.colorSensor.get("leftColor");
        rightColorSensor.setI2cAddress(rightColorI2c);
        leftColorSensor.setI2cAddress(leftColorI2c);
        rightColorSensor.enableLed(false);
        leftColorSensor.enableLed(false);

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);
    }

    public void allInit() {
        initWheels();
        initLauncher();
        initCollection();
        initRange();
        initTape();
        initButtons();
    }

    public boolean findVortex() throws InterruptedException {
        double vortexTargetWidth;
        if (launcherServo.getPosition() == launcherServoShortPos) {
            vortexTargetWidth = vortexTargetWidthShort;
        } else {
            vortexTargetWidth = vortexTargetWidthLong;
        }
        if (vortexX < vortexTargetX && seesVortex) {
            spinLeft(0.25);
            while (vortexX < vortexTargetX && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            sleep(50);
            if (vortexX > vortexTargetX && seesVortex) {
                spinRight(0.1);
                while (vortexX > vortexTargetX && seesVortex && opModeIsActive()) {
                    if (doCancelAutoLaunch())
                        return false;
                }
            }
        } else if (vortexX > vortexTargetX && seesVortex) {
            spinRight(0.25);
            while (vortexX > vortexTargetX && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            sleep(50);
            if (vortexX < vortexTargetX && seesVortex) {
                spinLeft(0.1);
                while (vortexX < vortexTargetX && seesVortex && opModeIsActive()) {
                    if (doCancelAutoLaunch())
                        return false;
                }
            }
        }

        if (vortexWidth < vortexTargetWidth && seesVortex) {
            goBackward(0.2);
            while (vortexWidth < vortexTargetWidth && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            sleep(50);
            if (vortexWidth > vortexTargetWidth && seesVortex) {
                goForward(0.1);
                while (vortexWidth > vortexTargetWidth && seesVortex && opModeIsActive()) {
                    if (doCancelAutoLaunch())
                        return false;
                }
            }
        } else if (vortexWidth > vortexTargetWidth && seesVortex) {
            goForward(0.2);
            while (vortexWidth > vortexTargetWidth && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            sleep(50);
            if (vortexWidth < vortexTargetWidth && seesVortex) {
                goBackward(0.1);
                while (vortexWidth < vortexTargetWidth && seesVortex && opModeIsActive()) {
                    if (doCancelAutoLaunch())
                        return false;
                }
            }
        }
        return fineVortexRotationAdjustment();
    }

    public boolean aimAtVortex() throws InterruptedException {
        if (vortexX < vortexTargetX - 0.1 && seesVortex) {
            spinLeft(0.3);
            while (vortexX < vortexTargetX - 0.1 && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
        } else if (vortexX > vortexTargetX + 0.1 && seesVortex) {
            spinRight(0.3);
            while (vortexX > vortexTargetX + 0.1 && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
        }
        sleep(150);
        fineVortexRotationAdjustment();

        if (vortexWidth < vortexValues[0][0] && seesVortex) {
            goBackward(0.25);
            while (vortexWidth < vortexValues[0][0] && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            fineVortexRotationAdjustment();
        } else if (vortexWidth > vortexValues[vortexValues.length - 1][0] && seesVortex) {
            goForward(0.25);
            while (vortexWidth > vortexValues[vortexValues.length - 1][0] && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
            fineVortexRotationAdjustment();
        }
        if (seesVortex) {
            launcherServo.setPosition(launcherPosFromVortexWidth(vortexWidth));
            sleep(500);
            return true;
        } else {
            return false;
        }
    }

    public double launcherPosFromVortexWidth(double vortexWidth) {
        if (vortexWidth < vortexValues[0][0]) {
            return vortexValues[0][1];//ERROR
        }
        for (int i = 0; i < vortexValues.length - 1; i++) {
            if (vortexWidth <= vortexValues[i + 1][0]) {
                return vortexValues[i][1] - ((vortexValues[i][1] - vortexValues[i + 1][1]) / (vortexValues[i + 1][0] - vortexValues[i][0])) * (vortexWidth - vortexValues[i][0]);
            }
        }
        return vortexValues[vortexValues.length - 1][1];//ERROR
    }

    public int launcherServoDelay() {
        return (int) (launchShortServoWaitTime + ((launchFarServoWaitTime - launchShortServoWaitTime) / (launcherServoFarPos - launcherServoShortPos)) * (launcherServo.getPosition() - launcherServoShortPos));
    }

    public boolean fineVortexRotationAdjustment() {
        if (vortexX < vortexTargetX - 0.02 && seesVortex) {
            spinLeft(0.1);
            while (vortexX < vortexTargetX - 0.02 && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
        } else if (vortexX > vortexTargetX + 0.02 && seesVortex) {
            spinRight(0.1);
            while (vortexX > vortexTargetX + 0.02 && seesVortex && opModeIsActive()) {
                if (doCancelAutoLaunch())
                    return false;
            }
        }
        stopRobot();
        return seesVortex;
    }

    public boolean doCancelAutoLaunch() {
        return (this instanceof CompetitionTeleOp) && ((Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2))) >= MIN_SPEED);
    }

    //OpenCV Stuff
    Camera camera;
    List<Camera.Area> meteringAreas = new ArrayList<>(1);

    Mat mRgba;
    Scalar mBlobColorRgba;
    Scalar mBlobColorHsv;
    ColorBlobDetector mDetector;
    Mat mSpectrum;
    Size SPECTRUM_SIZE;
    Scalar CONTOUR_COLOR;

    boolean seesVortex;
    double vortexWidth;
    double vortexHeight;
    double vortexX;
    double vortexY;

    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        if (FtcRobotControllerActivity.mOpenCvCameraView.isEnabled())
            FtcRobotControllerActivity.mOpenCvCameraView.disableView();
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage().sendToTarget();
        FtcRobotControllerActivity.mOpenCvCameraView.setCvCameraViewListener(cameraViewListener);
        FtcRobotControllerActivity.mOpenCvCameraView.enableView();
    }

    public void whenCameraViewStarts(int width, int height, Scalar blobColor, Scalar contourColor) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(200, 64);

        mBlobColorHsv = blobColor;
        CONTOUR_COLOR = contourColor;

        mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);

        mDetector.setHsvColor(mBlobColorHsv);

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        seesVortex = false;
        vortexWidth = 0;
        vortexHeight = 0;
        vortexX = 0;
        vortexY = 0;

        camera = FtcRobotControllerActivity.mOpenCvCameraView.getCamera();
        Camera.Area meteringArea = new Camera.Area(new Rect(-300, 700, 300, 1000), 1000);//The coordinates are mapped so that (-1000, -1000) is always the top-left corner of the current field of view, and (1000, 1000) is always the bottom-right corner of the current field of view.
        meteringAreas.clear();
        meteringAreas.add(meteringArea);
        Camera.Parameters parameters = camera.getParameters();
        parameters.setMeteringAreas(meteringAreas);
        camera.setParameters(parameters);
        camera.setPreviewCallback(new Camera.PreviewCallback() {
            @Override
            public void onPreviewFrame(byte[] data, Camera camera) {
                FtcRobotControllerActivity.mOpenCvCameraView.onPreviewFrame(data, camera);
                Camera.Parameters parameters = camera.getParameters();
                parameters.setMeteringAreas(meteringAreas);
                parameters.setExposureCompensation(FtcRobotControllerActivity.exposureValue);
                camera.setParameters(parameters);
            }
        });
    }

    public Mat processCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        mDetector.process(mRgba);
        List<MatOfPoint> contours = mDetector.getContours();
        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR, 4);

        Mat colorLabel = mRgba.submat(4, 68, 4, 68);
        colorLabel.setTo(mBlobColorRgba);

        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);

        boolean doesSeeVortex = false;
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
            if (width > minVortexWidth && width > maxWidth) {
                doesSeeVortex = true;
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
        seesVortex = doesSeeVortex;
        if (seesVortex) {
            meteringAreas.get(0).rect.set((int) ((vortexX - vortexWidth / 2) * 2000 - 1000), (int) ((vortexY) * 2000 - 1000), (int) ((vortexX + vortexWidth / 2) * 2000 - 1000), (int) ((vortexY + vortexHeight / 2) * 2000 - 1000));
        } else {
            meteringAreas.get(0).rect.set(-300, 700, 300, 1000);
        }

        return mRgba;
    }

    public Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
}