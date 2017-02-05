package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Secret Weapon (Tilt Slightly Left)", group = "OpMode")
public class RedSecretWeapon extends SecretWeapon {
    @Override
    public void spinInward(double speed, double degrees) throws InterruptedException {
        spinLeftDegrees(speed, degrees);
    }

    @Override
    public void spinOutward(double speed, double degrees) throws InterruptedException {
        spinRightDegrees(speed, degrees);
    }
}
