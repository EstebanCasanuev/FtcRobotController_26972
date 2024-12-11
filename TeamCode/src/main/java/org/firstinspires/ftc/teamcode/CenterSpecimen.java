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

@Autonomous(name = "CenterSpecimen")
public class CenterSpecimen extends LinearOpMode {

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    MecanumDrive m_Drive;
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

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx rightSlider, leftSlider;
    private MotorEx SlideMotion;


    //dashTelemetryPacket packet = new dashTelemetryPacket();


    //dashboard.senddashTelemetryPacket(packet);
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();



    public static final double TRACKWIDTH = 18;
    public static final double CENTER_WHEEL_OFFSET = 4;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    double xPos;
    double yPos;
    double zAngle;

    int autonomous = -1;

    @Override
    public void runOpMode() throws InterruptedException {



        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");

        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        PidX = new PIDController(PidX_P, PidX_I, PidX_D);
        PidY = new PIDController(PidY_P, PidY_I, PidY_D);
        PidZ = new PIDController(PidZ_P, PidZ_I, PidZ_D);


        frontLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        frontLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();

        rightSlider = new MotorEx(hardwareMap, "rightSlider");
        leftSlider = new MotorEx(hardwareMap, "leftSlider");

        SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        leftSlider.setInverted(true);



        odometry = new HolonomicOdometry(
                frontLeftMotor::getDistance,
                frontRightMotor::getDistance,
                rearRightMotor::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


        dashTelemetry.addData("Robot Position at Init: ", odometry.getPose());

        dashTelemetry.update();


        setBot_Setpoint(0, 0, 0);

        waitForStart();


        while (opModeIsActive()) {

            odometry.updatePose();

            xPos = odometry.getPose().getY();
            yPos = odometry.getPose().getX();
            zAngle = getAngle(odometry.getPose().getRotation().getDegrees());

            //dashTelemetry
            dashTelemetry.addData("X:" , xPos);
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


            switch(autonomous){
                case 0:
                    setBot_Setpoint(-23, 22.5, 0);
                    break;
                case 1:
                    setBot_Setpoint(6, 19, 0);
                    break;
                case 2:
                    setBot_Setpoint(13, 31, 0);
                    break;
                case 3:
                    setBot_Setpoint(14, 50, 0);
                    break;
                case 4:
                    setBot_Setpoint(23, 46, 0);
                    break;
                case 5:
                    setBot_Setpoint(23, 8, 0);
                    break;
                case 6:
                    setBot_Setpoint(23, 46.1, 0);
                    break;
                case 7:
                    setBot_Setpoint(31, 46, 0);
                    break;
                case 8:
                    setBot_Setpoint(31, 8, 0);
                    break;
                case 9:
                    setBot_Setpoint(31, 46.1, 0);
                    break;
                case 10:
                    setBot_Setpoint(7, 19, 0);
                    break;
                case 11:
                    setBot_Setpoint(7, 19, 90);
                    break;
                case 12:
                    setBot_Setpoint(-5, 2, 90);
                    break;
                case 13:
                    setBot_Setpoint(-7, 4, 90);
                    break;
                case 14:
                    setBot_Setpoint(-9, 10, 0);
                    break;
                case 15:
                    setBot_Setpoint(-29, 24, 0);
                    break;
                case 16:
                    setBot_Setpoint(7, 19.1, 0);
                    break;
                case 17:
                    setBot_Setpoint(7, 19.1, 90);
                    break;
                case 18:
                    setBot_Setpoint(-5, 2.1, 90);
                    break;
                case 19:
                    setBot_Setpoint(-7, 4, 90);
                    break;
                case 20:
                    setBot_Setpoint(-9, 10, 0);
                    break;
                case 21:
                    setBot_Setpoint(-34, 24, 0);
                    break;
                default:
                    m_Drive.driveRobotCentric(0, 0, 0);
                    break;
            }

            if (PidZ.getSetPoint() > 120 && PidZ.getSetPoint() < -240) {
                m_Drive.driveRobotCentric(
                        -PidX.calculate(odometry.getPose().getY())*0.5,
                        PidY.calculate(odometry.getPose().getX())*0.5,
                        -PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.35
                );

            }else if(PidZ.getSetPoint() == 90){
                m_Drive.driveRobotCentric(
                        PidY.calculate(odometry.getPose().getX())*0.5,
                        -PidX.calculate(odometry.getPose().getY())*0.5,
                        -PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.35
                );

            }else{

                m_Drive.driveRobotCentric(
                        -PidX.calculate(odometry.getPose().getY()) * 0.5,
                        -PidY.calculate(odometry.getPose().getX()) * 0.5,
                        -PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.35
                );
            }
        }
    }

    public void setBot_Setpoint(double X, double Y, double Z){
        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);

        if(atSetpoint(X, Y, Z)){

            autonomous += 1;
        }
    }

    /*public void setBot_Setpoint(double X, double Y, double Z, boolean ready){
        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);

        if(atSetpoint(X, Y, Z) && ready){
            autonomous += 1;
        }
    }*/

    public boolean atSetpoint(double X_Setpoint, double Y_Setpoint, double Z_Setpoint){
        return xPos > X_Setpoint - 2
                && xPos < X_Setpoint + 2
                && yPos > Y_Setpoint - 2
                && yPos < Y_Setpoint + 2
                && zAngle < Z_Setpoint + 5
                && zAngle > Z_Setpoint - 5;

    }

    public double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }


}