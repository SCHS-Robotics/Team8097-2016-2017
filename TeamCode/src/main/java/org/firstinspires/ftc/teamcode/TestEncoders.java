package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Test Encoders", group = "Test")
public class TestEncoders extends Autonomous {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double power = 0;
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

        spinRightDegrees(DEFAULT_SPIN_SPEED, 360);

        while (opModeIsActive()) {
            idle();
        }
    }
}
