package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Blue Beacons Don't Shoot Autonomous", group = "OpMode")
public class BlueBeaconsAutonomous extends BeaconsAutonomous {

    @Override
    public boolean shouldShoot() {
        return false;
    }

    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void fixPosAfterShooting() {
        //Do nothing
    }

    @Override
    public void turnToBeacons(double speed, double angle) throws InterruptedException {
        spinRightDegrees(speed, angle);
    }

    @Override
    public void turnAwayFromBeacons(double speed, double angle) throws InterruptedException {
        spinLeftDegrees(speed, angle);
    }

    @Override
    public void moveAcrossField(double speed) {
        goDiagonalBackwardLeft(speed);
    }

    @Override
    public void moveAcrossFieldDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalBackwardLeftDistance(speed, centimeters);
    }

    @Override
    public void moveAwayFromWallAfterCollecting(double speed, double centimeters) throws InterruptedException {
        goRightDistance(speed, centimeters);
    }

    @Override
    public void moveAlongBeaconWall(double speed) {
        goBackward(speed);
    }

    @Override
    public void moveAlongBeaconWallDistance(double speed, double centimeters) throws InterruptedException {
        goBackwardDistance(speed, centimeters);
    }

    @Override
    public void followBeaconWallDistance(double speed, double centimeters) throws InterruptedException {
        goForwardAndFollowWall(-speed, centimeters);
    }

    @Override
    public void fixPosForFindingTape() throws InterruptedException {
//        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 10);
    }

    @Override
    public void findTapeInward() throws InterruptedException {
        findTapeRight(-1);//left
    }

    @Override
    public void findTapeOutward() throws InterruptedException {
        findTapeRight(1);
    }

    @Override
    public void moveCorrectButtonFlap() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        if (leftBlue > rightBlue && leftRed < rightRed) {
            leftFlapServo.setPosition(leftFlapEndPos);
            rightFlapServo.setPosition(rightFlapInitPos);
        } else if (rightBlue > leftBlue && rightRed < leftRed) {
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        }
    }

    @Override
    public void setTeleOpAngle() {
        CompetitionTeleOp.currentAngle = 270;
    }

    @Override
    public Scalar getVortexColorHsv() {
        return blueHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return blueContrastRgb;
    }
}
