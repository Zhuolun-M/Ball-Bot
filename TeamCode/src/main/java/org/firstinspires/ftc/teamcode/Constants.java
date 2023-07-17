package org.firstinspires.ftc.teamcode;

public final class Constants {
    //All units are SI units (meters, kg, etc.)
    public final static double IMUhtGND = 0.458;
    public final static double IMUhtBall = 0.253;
    public final static double g = 9.81;
    public final static double BallMass = 0.624;
    public final static double Ballr = 0.1194;
    public final static double Omnir = 0.051;
    public final static double BallInertia = (0.6667 * BallMass * Ballr * Ballr);
    public final static double yzInertia = (0.6667 * BallMass * Ballr * Ballr/10);
    public final static double xyInertia = (0.6667 * BallMass * Ballr * Ballr/3.33);
    public final static double BodyInertia = (0.6667 * 5 * Ballr * Ballr);
    public final static double Bodyr = 0.0866;
    public final static double theta = 45;
    public final static double time_interval = 0.01;





}
