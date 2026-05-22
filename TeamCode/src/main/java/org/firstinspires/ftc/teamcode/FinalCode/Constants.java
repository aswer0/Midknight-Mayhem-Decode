package org.firstinspires.ftc.teamcode.FinalCode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;

@Config
public class Constants {
    public static Point blueGatePoint = new Point(9.5, 59.4);
    public static Point redGatePoint = new Point(130.2, 59.4);
    public static double gateAngle = 25;

    public static Point redResetPoint = new Point(122, 81);
    public static Point blueResetPoint = new Point(20, 81);

    public static double slowTransferOnTime = 200;
    public static double slowTransferOffTime = 280;
    public static double shootWaitTime = 650;

    public enum Alliance {
        red,
        blue,
    }
}
