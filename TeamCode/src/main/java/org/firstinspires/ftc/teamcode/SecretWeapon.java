package org.firstinspires.ftc.teamcode;

public abstract class SecretWeapon extends CompetitionAutonomous {
    @Override
    public int numParticles() {
        return 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        allInit();
        waitForStart();
        startLauncher();
        goBackwardDistance(DEFAULT_FORWARD_SPEED, 53);//60
        sleep(1000);
        //aimAtVortex();
        shoot();
        goForwardDistance(DEFAULT_FORWARD_SPEED, 51.5);//60
        spinInward(DEFAULT_SPIN_SPEED, 135);
        sleep(2200);
        goForwardDistance(1, 228);
        spinInward(DEFAULT_SPIN_SPEED, 90);
        if (this instanceof BlueSecretWeapon) {
            goForwardDistance(1, 109);
        } else {
            goForwardDistance(1, 119);
        }
        spinOutward(DEFAULT_SPIN_SPEED, 90);
        goForwardDistance(1, 60);

        while (opModeIsActive()) {
            idle();
        }
    }

    public abstract void spinInward(double speed, double degrees) throws InterruptedException;//left for red

    public abstract void spinOutward(double speed, double degrees) throws InterruptedException;//right for red
}
