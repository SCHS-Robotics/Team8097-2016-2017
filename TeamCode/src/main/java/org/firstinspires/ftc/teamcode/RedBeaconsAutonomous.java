package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Red Beacons Don't Shoot Autonomous", group = "OpMode")
public class RedBeaconsAutonomous extends BeaconsAutonomous {

    @Override
    public boolean shouldShoot() {
        return false;
    }

    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void fixPosAfterShooting() throws InterruptedException {
        spinRightDegrees(DEFAULT_SPIN_SPEED, 135);
    }

    @Override
    public void turnToBeacons(double speed, double angle) throws InterruptedException {
        spinLeftDegrees(speed, angle);
    }

    @Override
    public void turnAwayFromBeacons(double speed, double angle) throws InterruptedException {
        spinRightDegrees(speed, angle);
    }

    @Override
    public void moveAcrossField(double speed) {
        goDiagonalForwardLeft(speed);
    }

    @Override
    public void moveAcrossFieldDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalForwardLeftDistance(speed, centimeters);
    }

    @Override
    public void moveAwayFromWallAfterCollecting(double speed, double centimeters) throws InterruptedException {
        goLeftDistance(speed, centimeters);
    }

    @Override
    public void moveAlongBeaconWall(double speed) {
        goForward(speed);
    }

    @Override
    public void moveAlongBeaconWallDistance(double speed, double centimeters) throws InterruptedException {
        goForwardDistance(speed, centimeters);
    }

    @Override
    public void followBeaconWallDistance(double speed, double centimeters) throws InterruptedException {
        goForwardAndFollowWall(speed, centimeters);
    }

    @Override
    public void fixPosForFindingTape() throws InterruptedException {
//        goBackwardDistance(DEFAULT_FORWARD_SPEED, 5);
    }

    @Override
    public void findTapeInward() throws InterruptedException {
        findTapeRight();
    }

    @Override
    public void findTapeOutward() throws InterruptedException {
        findTapeLeft();
    }

    @Override
    public void moveCorrectButtonFlap() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        if (leftRed > rightRed && leftBlue < rightBlue) {
            leftFlapServo.setPosition(leftFlapEndPos);
            rightFlapServo.setPosition(rightFlapInitPos);
        } else if (rightRed > leftRed && rightBlue < leftBlue) {
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        } else { //testing without working beacon only
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        }
    }

    @Override
    public void setTeleOpAngle() {
        CompetitionTeleOp.currentAngle = 90;
    }

    @Override
    public Scalar getVortexColorHsv() {
        return redHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return redContrastRgb;
    }
}
