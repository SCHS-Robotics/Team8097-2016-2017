package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Red Just Shoot Autonomous", group = "OpMode")
public class RedJustShootAutonomous extends JustShootAutonomous {
    @Override
    public Scalar getVortexColorHsv() {
        return redHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return redContrastRgb;
    }
}
