package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    public void turn45ToBeacons() throws InterruptedException {
        spinRightDegrees(DEFAULT_SPIN_SPEED, 45);
    }

    @Override
    public void turn45backStraight() throws InterruptedException {
        spinLeftDegrees(DEFAULT_SPIN_SPEED, 45);
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
//        goForwardDistance(DEFAULT_FORWARD_SPEED, 5);
    }

    @Override
    public void findTapeInward() throws InterruptedException {
        findTapeLeft();
    }

    @Override
    public void findTapeOutward() throws InterruptedException {
        findTapeRight();
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
}
