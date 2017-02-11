package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Beacons Shoot 3 Autonomous", group = "OpMode")
public class RedBeaconsShoot3Autonomous extends RedBeaconsShootAutonomous {
    @Override
    public int numParticles() {
        return 3;
    }
}
