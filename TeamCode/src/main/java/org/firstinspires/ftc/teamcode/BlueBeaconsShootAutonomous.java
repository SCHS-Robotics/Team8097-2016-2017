package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Blue Beacons Shoot 1 Autonomous", group = "OpMode")
@Disabled
public class BlueBeaconsShootAutonomous extends BlueBeaconsAutonomous {
    @Override
    public boolean shouldShoot() {
        return true;
    }

    @Override
    public int numParticles() {
        return 1;
    }
}
