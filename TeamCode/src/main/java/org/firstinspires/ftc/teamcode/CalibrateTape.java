package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences.Editor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

@TeleOp(name = "Calibrate Tape", group = "Util")
public class CalibrateTape extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double frontValue;
    double backValue;
    int readings;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        readings = 0;
        FtcRobotControllerActivity.calibrateTape = false;
        FtcRobotControllerActivity.calibrateGround = false;
        frontValue = 0;
        backValue = 0;
        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
        frontTapeSensor.setI2cAddress(frontTapeI2c);
        frontTapeSensor.enableLed(true);
        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        backTapeSensor.setI2cAddress(backTapeI2c);
        backTapeSensor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());
            updateTelemetry();

            frontTapeSensor.enableLed(true);
            backTapeSensor.enableLed(true);
            logData("front light sensor", frontTapeSensor.alpha());
            logData("back light sensor", backTapeSensor.alpha());
            if (FtcRobotControllerActivity.calibrateTape) {
                if (readings < 100) {
                    readings++;
                    frontValue += frontTapeSensor.alpha();
                    backValue += backTapeSensor.alpha();
                    setButtonsClickable(false);
                } else {
                    readings = 0;
                    frontValue /= 100.0;
                    backValue /= 100.0;
                    FtcRobotControllerActivity.calibrateTape = false;
                    FtcRobotControllerActivity.setTapeText.obtainMessage(0, (int) (frontValue * 100), (int) (backValue * 100)).sendToTarget();
                    saveTapeValue();
                    frontValue = 0;
                    backValue = 0;
                }
            } else if (FtcRobotControllerActivity.calibrateGround) {
                if (readings < 100) {
                    readings++;
                    frontValue += frontTapeSensor.alpha();
                    backValue += backTapeSensor.alpha();
                    setButtonsClickable(false);
                } else {
                    readings = 0;
                    frontValue /= 100.0;
                    backValue /= 100.0;
                    FtcRobotControllerActivity.calibrateGround = false;
                    FtcRobotControllerActivity.setGroundText.obtainMessage(0, (int) (frontValue * 100), (int) (backValue * 100)).sendToTarget();
                    saveGroundValue();
                    frontValue = 0;
                    backValue = 0;
                }
            } else {
                setButtonsClickable(true);
            }
            sleep(1);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        setButtonsClickable(false);
    }

    private void setButtonsClickable(boolean clickable) {
        if (clickable) {
            FtcRobotControllerActivity.setButtonsClickable.obtainMessage().sendToTarget();
        } else {
            FtcRobotControllerActivity.setButtonsUnclickable.obtainMessage().sendToTarget();
        }
    }

    private void saveTapeValue() {
        Editor editor = FtcRobotControllerActivity.calibrationSP.edit();
        editor.putFloat("frontTapeValue", (float) frontValue);
        editor.putFloat("backTapeValue", (float) backValue);
        editor.apply();
    }

    private void saveGroundValue() {
        Editor editor = FtcRobotControllerActivity.calibrationSP.edit();
        editor.putFloat("frontGroundValue", (float) frontValue);
        editor.putFloat("backGroundValue", (float) backValue);
        editor.apply();
    }
}