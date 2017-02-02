package org.firstinspires.ftc.teamcode;

public abstract class SecretWeapon extends CompetitionAutonomous {
    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        allInit();
        waitForStart();
//        sleep(10000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 228);
        spinInward(DEFAULT_SPIN_SPEED, 90);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 119);
        spinOutward(DEFAULT_SPIN_SPEED, 90);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 76);

        while (opModeIsActive()) {
            idle();
        }
    }

    public abstract void spinInward(double speed, double degrees) throws InterruptedException;//left for red

    public abstract void spinOutward(double speed, double degrees) throws InterruptedException;//right for red
}
