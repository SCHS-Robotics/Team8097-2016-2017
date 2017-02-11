package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Beacons Shoot 3 Autonomous", group = "OpMode")
public class BlueBeaconsShoot3Autonomous extends BlueBeaconsShootAutonomous {
    @Override
    public int numParticles() {
        return 3;
    }
}
