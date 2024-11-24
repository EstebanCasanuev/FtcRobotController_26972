package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "OdometryTest")
public class TeleStaticRobotPose extends LinearOpMode {

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    MecanumDrive m_Drive;
    private HolonomicOdometry odometry;

    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    public static double PidX_P = 0.1;
    public static double PidX_I = 0;
    public static double PidX_D = 0;

    public static double PidY_P = 0.1;
    public static double PidY_I = 0;
    public static double PidY_D = 0;

    public static double PidZ_P = 0.1;
    public static double PidZ_I = 0;
    public static double PidZ_D = 0;


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

    @Override
    public void runOpMode() throws InterruptedException {

        int autonomous = 0;


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

            //dashTelemetry
            dashTelemetry.addData("X:" , odometry.getPose().getX());
            dashTelemetry.addData("Y:", odometry.getPose().getY());
            dashTelemetry.addData("Angulo:", odometry.getPose().getRotation().getDegrees());
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


            m_Drive.driveRobotCentric(
                    -PidX.calculate(odometry.getPose().getY())*0.5,
                    -PidY.calculate(odometry.getPose().getX())*0.5,
                    -PidZ.calculate(odometry.getPose().getHeading())*0.35
            );

            /*switch(autonomous){
                case 0:
                    setBot_Setpoint(-24, -24, 0);
                    if(odometry.getPose().getY() > -25 && odometry.getPose().getY() < -23 && odometry.getPose().getX() > -25 && odometry.getPose().getX() < -23){autonomous = 1;}
                    break;
                case 1:
                    setBot_Setpoint(-24, 24, 0);
                    if(odometry.getPose().getY() > -26 && odometry.getPose().getY() < -22 && odometry.getPose().getX() > 22 && odometry.getPose().getX() < 26){autonomous = 2;}
                    break;
                case 2:
                    setBot_Setpoint(24, 24, 0);
                    if(odometry.getPose().getY() > 22 && odometry.getPose().getY() < 26 && odometry.getPose().getX() > 22 && odometry.getPose().getX() < 26){autonomous = 3;}
                    break;

                case 3:
                    setBot_Setpoint(-24, 24, 0);
                    if(odometry.getPose().getY() > -26 && odometry.getPose().getY() < -22 && odometry.getPose().getX() > 22 && odometry.getPose().getX() < 26){autonomous = 4;}
                    break;

                case 4:
                    setBot_Setpoint(0, 0, 180);
                    if(odometry.getPose().getRotation().getDegrees() > 178 && odometry.getPose().getRotation().getDegrees() < 182){autonomous = 5;}
                    break;

                case 5:
                    setBot_Setpoint(-24, -24, 0);
                    if(odometry.getPose().getY() > -25 && odometry.getPose().getY() < -23 && odometry.getPose().getX() > -25 && odometry.getPose().getX() < -23){autonomous = 6;}
                    break;


            }*/

            setBot_Setpoint(0, 0, -130);

            /*m_Drive.driveRobotCentric(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );*/

            /*if(odometry.getPose().getX() < 25){
                m_Drive.driveRobotCentric(
                        0,
                        -0.6,
                        0
                );

            }else if(odometry.getPose().getX() > 25){
                m_Drive.driveRobotCentric(
                        0,
                        0.6,
                        0
                );
            }else{
                m_Drive.driveRobotCentric(
                        0,
                        0,
                        0
                );
            }*/
        }
    }
    public void setBot_Setpoint(double X, double Y, double Z){
        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);
    }
}
