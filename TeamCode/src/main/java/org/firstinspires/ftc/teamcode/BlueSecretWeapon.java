package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Secret Weapon (Tilt Slightly Left)", group = "OpMode")
public class BlueSecretWeapon extends SecretWeapon {
    @Override
    public void spinInward(double speed, double degrees) throws InterruptedException {
        spinRightDegrees(speed, degrees);
    }

    @Override
    public void spinOutward(double speed, double degrees) throws InterruptedException {
        spinLeftDegrees(speed, degrees);
    }
}
