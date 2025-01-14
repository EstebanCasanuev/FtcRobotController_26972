package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_P;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidX_D;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_P;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidY_D;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_P;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_I;
import static org.firstinspires.ftc.teamcode.Constants.Drivetrain_PIDContants.PidZ_D;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.OdometryConstants.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.D_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.I_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Slide_PIDConstants.P_SLIDEMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.D_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.I_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.Constants.Swing_PIDConstants.P_SWINGMOTION;
import static org.firstinspires.ftc.teamcode.OdometryRead.DISTANCE_PER_PULSE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

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

@Config
@TeleOp(name = "OdometryTest")
public class TeleStaticRobotPose extends LinearOpMode {

    BNO055IMU.Parameters myIMUparameters;


    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    MecanumDrive m_Drive;
    private HolonomicOdometry odometry;

    PIDController PidX;
    PIDController PidY;
    PIDController PidZ;

    public static double X_Setpoint = 0;
    public static double Y_Setpoint = 0;
    public static double Z_Setpoint = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dashboard.getTelemetry();


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


        setBot_Setpoint(0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {

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


            setBot_Setpoint(X_Setpoint, Y_Setpoint, Z_Setpoint);

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
    }
    public void setBot_Setpoint(double X, double Y, double Z){
        PidX.setSetPoint(X);
        PidY.setSetPoint(Y);
        PidZ.setSetPoint(Z);
    }

    public double getAngle(double ActualAngle) {
        return (ActualAngle+90) - 360 * Math.floor((ActualAngle+90)/360) - 90;
    }
}
