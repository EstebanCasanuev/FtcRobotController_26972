package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {

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

    public static double X_Setpoint = 0;
    public static double Y_Setpoint = 0;
    public static double Z_Setpoint = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();



    public static final double TRACKWIDTH = 11.25;
    public static final double CENTER_WHEEL_OFFSET = 0;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    public double xPos;
    public double yPos;
    public double zAngle;

    private int autonomous = -1;

    public DrivetrainSubsystem(){

        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");

        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        PidX = new PIDController(PidX_P, PidX_I, PidX_D);
        PidY = new PIDController(PidY_P, PidY_I, PidY_D);
        PidZ = new PIDController(PidZ_P, PidZ_I, PidZ_D);

        rearRightMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        rearLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();

        odometry = new HolonomicOdometry(
                ()->-rearLeftMotor.getDistance(),
                ()->-frontRightMotor.getDistance(),
                ()->rearRightMotor.getDistance(),
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


        dashTelemetry.addData("Robot Position at Init: ", odometry.getPose());

        dashTelemetry.update();


        setBot_Setpoint(()-> 0, ()-> 0, ()-> 0);





    }

    @Override
    public void periodic(){
        odometry.updatePose();

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

        xPos = odometry.getPose().getY();
        yPos = odometry.getPose().getX();
        zAngle = getAngle(odometry.getPose().getRotation().getDegrees());


        setBot_Setpoint( ()-> X_Setpoint, ()-> Y_Setpoint, ()-> Z_Setpoint);
    }

    public void setBot_Setpoint(DoubleSupplier X, DoubleSupplier Y,  DoubleSupplier Z){

        PidX.setSetPoint(X.getAsDouble());
        PidY.setSetPoint(Y.getAsDouble());
        PidZ.setSetPoint(Z.getAsDouble());

        if(atSetpoint(X.getAsDouble(), Y.getAsDouble(), Z.getAsDouble())){
            autonomous += 1;
        }
    }


    public boolean atSetpoint(double X_Setpoint, double Y_Setpoint, double Z_Setpoint){
        return xPos > X_Setpoint - 2
                && xPos < X_Setpoint + 2
                && yPos > Y_Setpoint - 2
                && yPos < Y_Setpoint + 2
                && zAngle < Z_Setpoint + 5
                && zAngle > Z_Setpoint - 5;

    }

    public void drive(Boolean Autonomous) {
        if(PidZ.getSetPoint() == -90){
            m_Drive.driveRobotCentric(
                    -PidY.calculate(odometry.getPose().getX())*0.5,
                    PidX.calculate(odometry.getPose().getY())*0.5,
                    -PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.35
            );

        } else if (PidZ.getSetPoint() > 120 && PidZ.getSetPoint() < -240) {
            m_Drive.driveRobotCentric(
                    -PidX.calculate(odometry.getPose().getY())*0.5,
                    PidY.calculate(odometry.getPose().getX())*0.5,
                    -PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.35
            );

        }else if(PidZ.getSetPoint() == 90){
            m_Drive.driveRobotCentric(
                    -PidY.calculate(odometry.getPose().getX())*0.5,
                    PidX.calculate(odometry.getPose().getY())*0.5,
                    PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees()))*0.35
            );

        } else {

            m_Drive.driveRobotCentric(
                    PidX.calculate(odometry.getPose().getY()) * 0.5,
                    PidY.calculate(odometry.getPose().getX()) * 0.5,
                    PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.35
            );
        }
    }

    public void drive(double Xspeed, double Yspeed, double Zspeed){
        m_Drive.driveRobotCentric(
                PidX.calculate(odometry.getPose().getY()) * 0.5,
                PidY.calculate(odometry.getPose().getX()) * 0.5,
                PidZ.calculate(getAngle(odometry.getPose().getRotation().getDegrees())) * 0.35
        );
    }

    private double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }

    public int getAutonomousStage(){
        return autonomous;
    }
}