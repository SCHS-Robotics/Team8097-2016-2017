package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Test Encoders", group = "Test")
public class TestEncoders extends Autonomous {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int waitTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();
//        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
//        backRightMotor = hardwareMap.dcMotor.get("backRight");
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

        waitForStart();
        runtime.reset();

        spinRightDegrees(DEFAULT_SPIN_SPEED, 45);
        sleep(1000);
        spinLeftDegrees(DEFAULT_SPIN_SPEED, 45);

//        goForward(DEFAULT_FORWARD_SPEED);

        while (opModeIsActive()) {
//            if (runtime.time() >= waitTime) {
//                runtime.reset();
//                logData("backLeft", getCurrentRpm(wheelEncoderPpr, backLeftMotor, waitTime));
//                encoderStartPos.put(backLeftMotor, Math.abs(backLeftMotor.getCurrentPosition()));
//                logData("backRight", getCurrentRpm(wheelEncoderPpr, backRightMotor, waitTime));
//                encoderStartPos.put(backRightMotor, Math.abs(backRightMotor.getCurrentPosition()));
//                logData("frontLeft", getCurrentRpm(wheelEncoderPpr, frontLeftMotor, waitTime));
//                encoderStartPos.put(frontLeftMotor, Math.abs(frontLeftMotor.getCurrentPosition()));
//                logData("frontRight", getCurrentRpm(wheelEncoderPpr, frontRightMotor, waitTime));
//                encoderStartPos.put(frontRightMotor, Math.abs(frontRightMotor.getCurrentPosition()));
//            }
            idle();
        }
    }
}
