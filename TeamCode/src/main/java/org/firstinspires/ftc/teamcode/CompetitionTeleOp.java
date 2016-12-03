package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "OpMode")
public class CompetitionTeleOp extends BaseOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pushButtonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    final int waitTime = 100;

    public static double currentAngle = 90;//initialized in autonomous based on which one is run.
    boolean spun = false;
    public final static int RIGHT = 0;
    public final static int FORWARD_RIGHT = 45;
    public final static int FORWARD = 90;
    public final static int FORWARD_LEFT = 135;
    public final static int LEFT = 180;
    public final static int BACKWARD_LEFT = 225;
    public final static int BACKWARD = 270;
    public final static int BACKWARD_RIGHT = 315;

    public final static double MIN_SPEED = 0;

    boolean launcherOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);

        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        pushButtonTime.reset();
        fixRpmTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());
            updateTelemetry();

            //Movement
            if (gamepad1.right_trigger > MIN_SPEED || gamepad1.left_trigger > MIN_SPEED) {
                if (!spun) {
                    resetWheelEncoders();
                }
                if (gamepad1.right_trigger > MIN_SPEED) {
                    spinRight(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > MIN_SPEED) {
                    spinLeft(gamepad1.left_trigger);
                }
                spun = true;
            } else {
                if (spun) {
                    double averageTicks = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()) / 4.0;
                    currentAngle += -averageTicks / TICKS_PER_DEGREE;
                }
                double joystickInputX = gamepad1.left_stick_x;
                double joystickInputY = -gamepad1.left_stick_y;
                double magnitude = Math.sqrt(Math.pow(joystickInputX, 2) + Math.pow(joystickInputY, 2));
                double angle = Math.toDegrees(Math.atan2(joystickInputY, joystickInputX));
                angle -= (currentAngle - 90);
                if (magnitude > MIN_SPEED) {
                    goDirection(magnitude, angle);
                } else {
                    stopRobot();
                }
                spun = false;
            }

            if (gamepad1.right_stick_button) {
                currentAngle = 90;
                logData("Angle calibrated!", "Angle calibrated!");
            }

            //Button Pushers
            if (gamepad1.left_bumper) {
                pushButtonTime.reset();
                leftFlapServo.setPosition(leftFlapEndPos);
                rightFlapServo.setPosition(rightFlapInitPos);
            } else if (gamepad1.right_bumper) {
                pushButtonTime.reset();
                rightFlapServo.setPosition(rightFlapEndPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }
            if (pushButtonTime.time() >= 500) {
                rightFlapServo.setPosition(rightFlapInitPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }

            if (gamepad1.a) {
                if (!launcherOn) {
                    startLauncher(launcherMaxRpm);
                    launcherOn = true;
                } else {
                    leftLaunchMotor.setPower(0);
                    rightLaunchMotor.setPower(0);
                    launcherOn = false;
                }
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void goDirection(double magnitude, double angle) throws InterruptedException {
        if (angleIsNearAngle(angle, RIGHT)) {
            goRight(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD_RIGHT)) {
            goDiagonalForwardRight(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD)) {
            goForward(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD_LEFT)) {
            goDiagonalForwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, LEFT)) {
            goLeft(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD_LEFT)) {
            goDiagonalBackwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD)) {
            goBackward(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD_RIGHT)) {
            goDiagonalBackwardRight(magnitude);
        } else {
            stopRobot();
            logData("ERROR!!!", "ERROR!!!");
        }
    }

    public boolean angleIsNearAngle(double angle1, double angle2) {
        while (angle1 >= 360) {
            angle1 -= 360;
        }
        while (angle1 < 0) {
            angle1 += 360;
        }
        while (angle2 >= 360) {
            angle2 -= 360;
        }
        while (angle2 < 0) {
            angle2 += 360;
        }
        double diff = Math.abs(angle2 - angle1);
        return diff < 45.0 / 2 || diff > 360 - 45.0 / 2;
    }
}
