package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DrivetrainSubsystem extends SubsystemBase {

    Motor frontRightMotor;
    Motor rearRightMotor;
    Motor frontLeftMotor;
    Motor rearLeftMotor;

    MecanumDrive m_Drive;

    public static final double TRACKWIDTH = 14.7; // The lateral distance between the left and right odometers
    public static final double CENTER_WHEEL_OFFSET = -2.1;
    public static final double WHEEL_DIAMETER = 1.377;
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx leftOdometer, rightOdometer, centerOdometer;

    HolonomicOdometry m_robotOdometry = new HolonomicOdometry(
            ()->leftOdometer.getDistance(),
            ()->rightOdometer.getDistance(),
            ()->centerOdometer.getDistance(),
            TRACKWIDTH, CENTER_WHEEL_OFFSET
    );

    OdometrySubsystem Odometry = new OdometrySubsystem(m_robotOdometry);



    // pass the odometry object into the subsystem constructor
    public DrivetrainSubsystem(){

        leftOdometer = new MotorEx(hardwareMap, "leftOdometer");
        rightOdometer =  new MotorEx(hardwareMap, "rightOdometer");
        centerOdometer =  new MotorEx(hardwareMap, "centerOdometer");


        m_Drive = new MecanumDrive(frontLeftMotor,frontRightMotor, rearLeftMotor, rearRightMotor);
        frontRightMotor = new Motor(hardwareMap, "frontRightMotor");
        rearRightMotor = new Motor(hardwareMap, "rearRightMotor");
        frontLeftMotor = new Motor(hardwareMap, "frontLeftMotor");
        rearLeftMotor = new Motor(hardwareMap, "rearLeftMotor");



    }

    /*@Override
    public void initialize(){

    }*/

    @Override
    public void periodic() {
        super.periodic();
    }

    public Pose2d Coordenates() {
        return Odometry.getPose();
    }

    public static void drive(Double Xspeed, Double Yspeed, Double Zspeed){
        m_Drive.driveRobotCentric(Xspeed, Yspeed, Zspeed);
    }
}
