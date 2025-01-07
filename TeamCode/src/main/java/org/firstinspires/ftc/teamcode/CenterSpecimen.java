package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

@Autonomous(name = "CenterSpecimen")
public class CenterSpecimen extends LinearOpMode {


    
    DrivetrainSubsystem m_Drive;
    private HolonomicOdometry odometry;

    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    public static double PidX_P = 0.20;
    public static double PidX_I = 0;
    public static double PidX_D = 0;

    public static double PidY_P = 0.25;
    public static double PidY_I = 0;
    public static double PidY_D = 0;

    public static double PidZ_P = 0.1;
    public static double PidZ_I = 0;
    public static double PidZ_D = 0;

    public static double P_SLIDEMOTION= 0.00;
    public static double I_SLIDEMOTION= 0.00;
    public static double D_SLIDEMOTION= 0.00;

    public static double P_SWINGMOTION= 0.00;
    public static double I_SWINGMOTION= 0.00;
    public static double D_SWINGMOTION= 0.00;

    //private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    /*private MotorEx rightSlider, leftSlider;
    private MotorEx SlideMotion;*/


    //dashTelemetryPacket packet = new dashTelemetryPacket();


    //dashboard.senddashTelemetryPacket(packet);
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();



    public static final double TRACKWIDTH = 11.25;
    public static final double CENTER_WHEEL_OFFSET = 0;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    double xPos;
    double yPos;
    double zAngle;

    int autonomous = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        dashTelemetry.update();


        m_Drive.setBot_Setpoint(()-> 0.1, ()-> 0.1, ()-> 0.1);

        waitForStart();


        while (opModeIsActive()) {

            autonomous = m_Drive.getAutonomousStage();

            odometry.updatePose();

            xPos = odometry.getPose().getY();
            yPos = odometry.getPose().getX();
            zAngle = getAngle(odometry.getPose().getRotation().getDegrees());

            //dashTelemetry
            dashTelemetry.addData("X:", xPos);
            dashTelemetry.addData("Y:", yPos);
            dashTelemetry.addData("Angulo:", zAngle);
            dashTelemetry.addData("X Setpoint:", PidX.getSetPoint());
            dashTelemetry.addData("Y Setpoint:", PidY.getSetPoint());
            dashTelemetry.addData("Z Setpoint:", PidZ.getSetPoint());

            dashTelemetry.addData("X Drivetrain Velocity:",
                    PidX.calculate(odometry.getPose().getY()));

            dashTelemetry.addData("Y Drivetrain Velocity:",
                    PidY.calculate(odometry.getPose().getX()));

            dashTelemetry.addData("Z Drivetrain Velocity:",
                    PidZ.calculate(odometry.getPose().getHeading()));

            dashTelemetry.addData("PIDX P", PidX.getP());
            dashTelemetry.addData("PIDX I", PidX.getI());
            dashTelemetry.addData("PIDX D", PidX.getD());

            dashTelemetry.addData("PIDY P", PidY.getP());
            dashTelemetry.addData("PIDY I", PidY.getI());
            dashTelemetry.addData("PIDY D", PidY.getD());

            dashTelemetry.addData("PIDZ P", PidZ.getP());
            dashTelemetry.addData("PIDZ I", PidZ.getI());
            dashTelemetry.addData("PIDZ D", PidZ.getD());

            dashTelemetry.update();


            switch (autonomous) {
                case 0:
                    m_Drive.setBot_Setpoint(()-> -23, ()-> 22.5, ()-> 0);
                    break;
                case 1:
                    m_Drive.setBot_Setpoint(()-> 6, ()-> 19, ()-> 0);
                    break;
                case 2:
                    m_Drive.setBot_Setpoint(()-> 13, ()-> 31, ()-> 0);
                    break;
                case 3:
                    m_Drive.setBot_Setpoint(()-> 14, ()-> 50, ()-> 0);
                    break;
                case 4:
                    m_Drive.setBot_Setpoint(()-> 23, ()-> 46, ()-> 0);
                    break;
                case 5:
                    m_Drive.setBot_Setpoint(()-> 23, ()-> 8, ()-> 0);
                    break;
                case 6:
                    m_Drive.setBot_Setpoint(()-> 23, ()-> 46.1, ()-> 0);
                    break;
                case 7:
                    m_Drive.setBot_Setpoint(()-> 31, ()-> 46, ()-> 0);
                    break;
                case 8:
                    m_Drive.setBot_Setpoint(()-> 31, ()-> 8, ()-> 0);
                    break;
                case 9:
                    m_Drive.setBot_Setpoint(()-> 31, ()-> 46.1, ()-> 0);
                    break;
                case 10:
                    m_Drive.setBot_Setpoint(()-> 7, ()-> 19, ()-> 0);
                    break;
                case 11:
                    m_Drive.setBot_Setpoint(()-> 7, ()-> 19, ()-> 90);
                    break;
                case 12:
                    m_Drive.setBot_Setpoint(()-> -5, ()-> 2, ()-> 90);
                    break;
                case 13:
                    m_Drive.setBot_Setpoint(()-> -7, ()-> 4, ()-> 90);
                    break;
                case 14:
                    m_Drive.setBot_Setpoint(()-> -9, ()-> 10, ()-> 0);
                    break;
                case 15:
                    m_Drive.setBot_Setpoint(()-> -29, ()-> 24, ()-> 0);
                    break;
                case 16:
                    m_Drive.setBot_Setpoint(()-> 7, ()-> 19.1, ()-> 0);
                    break;
                case 17:
                    m_Drive.setBot_Setpoint(()-> 7, ()-> 19.1, ()-> 90);
                    break;
                case 18:
                    m_Drive.setBot_Setpoint(()-> -5, ()-> 2.1, ()-> 90);
                    break;
                case 19:
                    m_Drive.setBot_Setpoint(()-> -7, ()-> 4, ()-> 90);
                    break;
                case 20:
                    m_Drive.setBot_Setpoint(()-> -9, ()-> 10, ()-> 0);
                    break;
                case 21:
                    m_Drive.setBot_Setpoint(()-> -34, ()-> 24, ()-> 0);
                    break;
                default:
                    m_Drive.drive(0, 0, 0);
                    break;
            }
            m_Drive.drive(true);
        }
    }
    public double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }
}