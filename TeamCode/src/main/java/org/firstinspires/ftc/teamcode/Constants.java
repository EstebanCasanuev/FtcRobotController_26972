package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

@Config
public class Constants {

    public class SwingSetpoints {
        public static final int UP = 1950;
        public static final int MIDDLE = 1700;
        public static final int DOWN = 250;
    }

    public class SlideSetpoints {
        public static final int ALL_THE_WAY = -4000;
        public static final int ALL_IN = -80;
        public static final int IN_THE_MIDDLE = -2300;
    }

    public class Drivetrain_PIDContants {
        public static final double PidX_P = 0.20;
        public static final double PidX_I = 0;
        public static final double PidX_D = 0;

        public static final double PidY_P = 0.25;
        public static final double PidY_I = 0;
        public static final double PidY_D = 0;

        public static final double PidZ_P = 0.1;
        public static final double PidZ_I = 0;
        public static final double PidZ_D = 0;
    }
    

    public class Swing_PIDConstants {
        public static final double P_SWINGMOTION= 0.005;
        public static final double I_SWINGMOTION= 0.00;
        public static final double D_SWINGMOTION= 0.000001;

        public static final double DOWN_P_SWINGMOTION= 0.000000001;
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
