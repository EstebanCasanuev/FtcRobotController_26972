package org.firstinspires.ftc.teamcode;

public class Constants {

    public class SwingSetpoints {
        public static final int UP = 1885;
        public static final int THREE_QUARTERS = 1560;
        public static final int DOWN = 70;
    }

    public class SlideSetpoints {
        public static final int ALL_THE_WAY = -4000;
        public static final int ALL_IN = -10;
        public static final int IN_THE_MIDDLE = -1800;

        public static final int HORIZONTAL_ALL_THE_WAY = -3000;
    }

    public class AutonomousSetpoint{
        public static final int AUTONOMOUS_FIRSTSWING_SETPOINT = 1600;
        public static final int AUTONOMOUS_SECONDSWING_SETPOINT = 1200;
        public static final int AUTONOMOUS_SWINGTAKEIN_SETPOINT = 700;

    }

    public class Drivetrain_PIDContants {
        public static final double PidX_P = 0.20;
        public static final double PidX_I = 0.00001;
        public static final double PidX_D = 0.001;

        public static final double PidY_P = 0.18;
        public static final double PidY_I = 0.0000008;
        public static final double PidY_D = 0.001;

        public static final double PidZ_P = 0.13;
        public static final double PidZ_I = 0.0000000000000008;
        public static final double PidZ_D = 0.00001;
    }
    

    public class Swing_PIDConstants {
        public static final double P_SWINGMOTION= 0.007;
        public static final double I_SWINGMOTION= 0.00;
        public static final double D_SWINGMOTION= 0.000001;

        public static final double DOWN_P_SWINGMOTION= 0.00008;
    }

    public class Slide_PIDConstants {
        public static final double P_SLIDEMOTION= 0.015;
        public static final double I_SLIDEMOTION= 0.00;
        public static final double D_SLIDEMOTION= 0.00;
    }
    
    public class OdometryConstants{
        public static final double TRACKWIDTH = 11.25;
        public static final double CENTER_WHEEL_OFFSET = 0;
        public static final double WHEEL_DIAMETER = 1.25;
        public static final double TICKS_PER_REV = 2000;
    }
}
