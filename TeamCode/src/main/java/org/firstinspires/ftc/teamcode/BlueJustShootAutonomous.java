package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Blue Just Shoot Autonomous", group = "OpMode")
public class BlueJustShootAutonomous extends JustShootAutonomous {
    @Override
    public Scalar getVortexColorHsv() {
        return blueHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return blueContrastRgb;
    }
}
