package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Scalar;

@TeleOp(name = "Red TeleOp", group = "OpMode")
public class RedTeleOp extends CompetitionTeleOp {
    @Override
    public Scalar getVortexColorHsv() {
        return redHsv;
    }

    @Override
    public Scalar getVortexOutlineColorRgb() {
        return redContrastRgb;
    }
}
