package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Red Cap Ball Autonomous", group = "OpMode")
public class RedCapBallAutonomous extends CapBallAutonomous {
    @Override
    public Scalar getVortexColorHsv() {
        return redHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return redContrastRgb;
    }
}
