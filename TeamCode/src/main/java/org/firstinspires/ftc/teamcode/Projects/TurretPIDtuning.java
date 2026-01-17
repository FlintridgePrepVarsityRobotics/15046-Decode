package org.firstinspires.ftc.teamcode.Projects;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretPIDtuning {

    public static double kP = 0.010;
    public static double kI = 0.000;
    public static double kD = 0.001;


    public static double MAX_POWER = 0.6;


    public static double POSITION_TOLERANCE = 1.0;

    public static double TICKS_PER_DEGREE = 10.0;


    public static double TARGET_ANGLE_DEG = 90.0;

}
