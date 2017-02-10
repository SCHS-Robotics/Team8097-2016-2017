package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Scalar;

@TeleOp(name = "Blue TeleOp", group = "OpMode")
public class BlueTeleOp extends CompetitionTeleOp {
    @Override
    public Scalar getVortexColorHsv() {
        return blueHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return blueContrastRgb;
    }
}
