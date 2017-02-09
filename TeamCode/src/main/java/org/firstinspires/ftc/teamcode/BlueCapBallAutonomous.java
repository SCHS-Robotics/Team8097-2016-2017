package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous(name = "Blue Cap Ball Autonomous", group = "OpMode")
public class BlueCapBallAutonomous extends CapBallAutonomous {
    @Override
    public Scalar getVortexColorHsv() {
        return blueHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return blueContrastRgb;
    }
}
