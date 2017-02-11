package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Red Beacons Shoot 1 Autonomous", group = "OpMode")
@Disabled
public class RedBeaconsShootAutonomous extends RedBeaconsAutonomous {
    @Override
    public boolean shouldShoot() {
        return true;
    }

    @Override
    public int numParticles() {
        return 1;
    }

    @Override
    public void turnToBeacons(double speed, double angle) throws InterruptedException {
        //Do nothing
    }
}
