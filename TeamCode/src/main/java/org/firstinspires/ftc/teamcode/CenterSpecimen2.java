package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "CenterSpecimen2")
public class CenterSpecimen2 extends LinearOpMode {

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    MecanumDrive m_Drive;
    private Servo servoder, servoizq, servocanasta, servo80;
    private CRServo servopinza;
    private DcMotor poleasmotor, poleasmotor2;
    private HolonomicOdometry odometry;
    private ElapsedTime TIEMPO = new ElapsedTime();
    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    PIDController motorpoleaspid;
    PIDController motorpoleaspid2;

    public static double PidX_P = 0.15;
    public static double PidX_I = 0;
    public static double PidX_D = 0;

    public static double PidY_P = 0.15;
    public static double PidY_I = 0;
    public static double PidY_D = 0;

    public static double PidZ_P = 0.1;
    public static double PidZ_I = 0;
    public static double PidZ_D = 0;

    public static double P_SLIDEMOTION = 0.00;
    public static double I_SLIDEMOTION = 0.00;
    public static double D_SLIDEMOTION = 0.00;

    public static double P_SWINGMOTION = 0.00;
    public static double I_SWINGMOTION = 0.00;
    public static double D_SWINGMOTION = 0.00;

    /*private MotorEx rightSlider, leftSlider;
    private MotorEx SlideMotion;*/


    //dashTelemetryPacket packet = new dashTelemetryPacket();


    //dashboard.senddashTelemetryPacket(packet);
    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();


    public static final double TRACKWIDTH = 13.75;
    public static final double CENTER_WHEEL_OFFSET = 1.250;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    double xPos;
    double yPos;
    double zAngle;
    double SERVO80 = 0;
    double SERVOIZQ = 0.37;
    double SERVODER = 0.63;
    double SERVOCAN = 0.6;
    int SERVOPINZA = 0;
    private static final int MOTOR_DE_LAS_POLEAS_POS_UP = -18350;
    private static final int MOTOR_DE_LAS_POLEAS_POS_MIDDLE = -10350;
    private static final int MOTOR_DE_LAS_POLEAS_POS_ESTACIONADO = -6000;

    int autonomous = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        servo80 = hardwareMap.get(Servo.class, "servo 80");
        servoder = hardwareMap.get(Servo.class, "servo der");
        servoizq = hardwareMap.get(Servo.class, "servo izq");
        servocanasta = hardwareMap.get(Servo.class, "servo canasta");
        servopinza = hardwareMap.get(CRServo.class, "servo pinza");

        servoder.setPosition(SERVODER);
        servoizq.setPosition(SERVOIZQ);
        servocanasta.setPosition(SERVOCAN);
        servo80.setPosition(SERVO80);
        servopinza.setPower(SERVOPINZA);

        motorpoleaspid = new PIDController(0.3, 0, 0.01);
        motorpoleaspid2 = new PIDController(0.3, 0, 0.01);

        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");

        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        PidX = new PIDController(PidX_P, PidX_I, PidX_D);
        PidY = new PIDController(PidY_P, PidY_I, PidY_D);
        PidZ = new PIDController(PidZ_P, PidZ_I, PidZ_D);


        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        rearRightMotor.resetEncoder();
        frontLeftMotor.resetEncoder();
        rearLeftMotor.resetEncoder();

        poleasmotor = hardwareMap.get(DcMotor.class, "poleas motor");
        poleasmotor2 = hardwareMap.get(DcMotor.class, "poleas motor 2");

        poleasmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poleasmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        poleasmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poleasmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry = new HolonomicOdometry(
                frontLeftMotor::getDistance,
                rearRightMotor::getDistance,
                rearLeftMotor::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


        dashTelemetry.addData("Robot Position at Init: ", odometry.getPose());

        dashTelemetry.update();


        setBot_Setpoint(0, 0, 0);

        waitForStart();


        while (opModeIsActive()) {

            servoder.setPosition(SERVODER);
            servoizq.setPosition(SERVOIZQ);
            servocanasta.setPosition(SERVOCAN);
            servo80.setPosition(SERVO80);
            servopinza.setPower(SERVOPINZA);

            odometry.updatePose();

            xPos = odometry.getPose().getY();
            yPos = odometry.getPose().getX();
            zAngle = -odometry.getPose().getRotation().getDegrees();

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
            telemetry.addData("X", xPos);
            telemetry.addData("X", yPos);
            telemetry.addData("X querer", (odometry.getPose().getX()));
            telemetry.addData("X querer", (odometry.getPose().getY()));
            telemetry.addData("servo pos", servocanasta.getPosition());
            telemetry.addData("Case", autonomous);
            dashTelemetry.update();
            telemetry.update();

            switch (autonomous) {
                case 0:
                    setBot_Setpoint(0, 6, 0);
                    break;
                case 1:
                    setBot_Setpoint(-34, 6, 0);
                    break;
                case 2:
                    setBot_Setpoint(-31, 5, -45);
                    break;
                case 3:
                    setBot_Setpoint(-34, 5, -45);
                    TIEMPO.reset();
                    break;
                case 4:
                    setPoleas_Setpoint(MOTOR_DE_LAS_POLEAS_POS_UP);
                    TIEMPO.reset();
                    break;
                case 5:
                    SERVOCAN = 0.1;
                    if ((TIEMPO.milliseconds()) > 1200) {
                        autonomous += 1;
                    }
                    break;
                case 6:
                    SERVOCAN = 0.9;
                    setBot_Setpoint(-36, 8, 0);
                    setPoleas_Setpoint(0);
                    break;
                case 7:
                    setBot_Setpoint(-12, 8, 0);
                    break;
                case 8:
                    setBot_Setpoint(-12, 55, 0);
                    break;
                case 9:
                    setBot_Setpoint(-24, 55, 0);
                    break;
                case 10:
                    setBot_Setpoint(-28, 8, 0);
                    break;
                case 11:
                    setBot_Setpoint(-24, 55, 0);
                    break;
                case 12:
                    setBot_Setpoint(-34, 55, 0);
                    break;
                case 13:
                    setBot_Setpoint(-34, 8, 0);
                    setPoleas_Setpoint(MOTOR_DE_LAS_POLEAS_POS_ESTACIONADO);
                    break;
                case 14:
                    setBot_Setpoint(-34, 51, 0);
                    break;
                case 15:
                    setBot_Setpoint(-2, 54 , 90);
                    break;
                default:
                    m_Drive.driveRobotCentric(0, 0, 0);
                    break;
            }

            poleasmotor.setPower(motorpoleaspid.calculate(poleasmotor.getCurrentPosition()));
            poleasmotor2.setPower(motorpoleaspid2.calculate(poleasmotor2.getCurrentPosition()));

            if (PidZ.getSetPoint() == 180) {
                m_Drive.driveRobotCentric(
                        -PidX.calculate(odometry.getPose().getY()) * 0.6,
                        -PidY.calculate(odometry.getPose().getX()) * 0.6,
                        PidZ.calculate(-odometry.getPose().getRotation().getDegrees()) * 0.35
                );

            } else if (PidZ.getSetPoint() == 90) {
                m_Drive.driveRobotCentric(
                        -PidY.calculate(odometry.getPose().getX()) * 0.6,
                        PidX.calculate(odometry.getPose().getY()) * 0.6,
                        PidZ.calculate(-odometry.getPose().getRotation().getDegrees()) * 0.35
                );

            } else {

                m_Drive.driveRobotCentric(
                        -PidX.calculate(odometry.getPose().getY()) * 0.6,
                        -PidY.calculate(odometry.getPose().getX()) * 0.6,
                        PidZ.calculate(-odometry.getPose().getRotation().getDegrees()) * 0.35
                );
            }
        }
    }

    public void setBot_Setpoint(double X, double Y, double Z) {
        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);

        if (atSetpoint(X, Y, Z)) {

            autonomous += 1;
        }
    }

    public void setPoleas_Setpoint(int motorSetpoint) {
        motorpoleaspid.setSetPoint(motorSetpoint);
        motorpoleaspid2.setSetPoint(-motorSetpoint);

        if (atSetpoint(motorSetpoint)) {
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

    public boolean atSetpoint(double X_Setpoint, double Y_Setpoint, double Z_Setpoint) {
        return xPos > X_Setpoint - 2
                && xPos < X_Setpoint + 2
                && yPos > Y_Setpoint - 2
                && yPos < Y_Setpoint + 2
                && zAngle < Z_Setpoint + 5
                && zAngle > Z_Setpoint - 5;

    }

    public boolean atSetpoint(double Setpoint1) {
        return poleasmotor.getCurrentPosition() > Setpoint1 - 15
                && poleasmotor.getCurrentPosition() < Setpoint1 + 15;

    }

    public double getAngle(double ActualAngle) {
        return (ActualAngle + 90) - 360 * Math.floor((ActualAngle + 90) / 360) - 90;
    }

}