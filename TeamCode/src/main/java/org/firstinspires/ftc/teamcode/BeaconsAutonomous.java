package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public abstract class BeaconsAutonomous extends CompetitionAutonomous {

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final int closeToWallDistance = 25;//centimeters
    final int beforePushingButtonDistance = 10;//centimeters

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();
        loadTapeValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        setTeleOpAngle();

        if (shouldShoot()) {
            shoot();
            goBackwardDistance(DEFAULT_FORWARD_SPEED, 10);
            fixPosAfterShooting();
        } else {
            moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 10);
        }
        turn45ToBeacons();
        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 130);
        turn45backStraight();
        goToBeaconWall(DEFAULT_SIDEWAYS_SPEED, closeToWallDistance);
        fixPosForFindingTape();//Does nothing right now
        findTapeInward();
        alignWithWall();
        goForwardDistance(0.4, 4);//moveAlongBeaconWall
        findTapeLeft();//findTapeOutward
        pushButton();
        moveAlongStartWallDistance(-DEFAULT_SIDEWAYS_SPEED, 12.5);
        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 97);
        findTapeInward();
        alignWithWall();
        goForwardDistance(0.4, 4);//moveAlongBeaconWall
        findTapeLeft();//findTapeOutward
        pushButton();

        while (opModeIsActive()) {
            idle();
        }
    }

    public abstract boolean shouldShoot();

    public abstract void fixPosAfterShooting() throws InterruptedException;

    public abstract void turn45ToBeacons() throws InterruptedException;

    public abstract void turn45backStraight() throws InterruptedException;

    public void goToBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        while (getRangeDistance() > cmFromWall && opModeIsActive()) {
            moveAlongStartWall(speed);
        }
        stopRobot();
    }

    public void goAwayFromBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        while (getRangeDistance() < cmFromWall && opModeIsActive()) {
            moveAlongStartWall(-speed);
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
        goAwayFromBeaconWall(0.34, beforePushingButtonDistance);
        goToBeaconWall(0.34, beforePushingButtonDistance);
        stopRobot();
    }

    //These movements are with respect to the field. Different for red and blue because they mirror each other.
    public abstract void moveAcrossField(double power);

    public abstract void moveAcrossFieldDistance(double power, double centimeters) throws InterruptedException;

    public void moveAlongStartWall(double power) {
        goLeft(power);
    }

    public void moveAlongStartWallDistance(double power, double centimeters) throws InterruptedException {
        goLeftDistance(power, centimeters);
    }

    public abstract void moveAlongBeaconWall(double power);

    public abstract void moveAlongBeaconWallDistance(double power, double centimeters) throws InterruptedException;

    public abstract void fixPosForFindingTape() throws InterruptedException;

    public void findTapeRight() throws InterruptedException {
        sleep(250);
        boolean firstTime = true;
        int i = 0;
        while (i < 1 && (frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (!firstTime) {
                goBackwardDistance(0.3, 5);
            }
            firstTime = false;
            while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                logData("light", frontTapeSensor.alpha());
                updateTelemetry();
                goForward(0.2);
            }
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveLeftWheelsForward(0.2);
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveRightWheelsForward(0.2);
                }
            }
            i++;
        }
        stopRobot();
    }

    public void findTapeLeft() throws InterruptedException {
        sleep(250);
        boolean firstTime = true;
        int i = 0;
        while (i < 1 && (frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (!firstTime) {
                goForwardDistance(0.3, 5);
            }
            firstTime = false;
            while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                logData("light", frontTapeSensor.alpha());
                updateTelemetry();
                goBackward(0.2);
            }
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveLeftWheelsBackward(0.2);
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveRightWheelsBackward(0.2);
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
//        do {
        moveCorrectButtonFlap();
        sleep(250);
        resetButtonFlaps();
//            sleep(250);
//        } while (!buttonIsPressed());
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
}