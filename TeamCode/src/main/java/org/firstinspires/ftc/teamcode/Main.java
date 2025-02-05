package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Subsystems.SensorColorSubsystem;

@Config
@TeleOp(name = "Main")
public class Main extends LinearOpMode {

    public static double P_SLIDEMOTION= 0.1;
    public static double I_SLIDEMOTION= 0.00;
    public static double D_SLIDEMOTION= 0.00;

    public static double P_SWINGMOTION= 0.1;
    public static double I_SWINGMOTION= 0.00;
    public static double D_SWINGMOTION= 0.00;

    //private HolonomicOdometry odometry;

    OdometryRead odometry;

    public static final double TRACKWIDTH = 11.25;
    public static final double CENTER_WHEEL_OFFSET = 0;
    public static final double WHEEL_DIAMETER = 1.25;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 2000;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx rightSlider, leftSlider;
    //private MotorEx SlideMotion;

    SensorColorSubsystem Sensor_Color = new SensorColorSubsystem();

    CRServo Intake;

    //PIDController SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
    //PIDController SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

    MecanumDrive m_Drive;
    @Override
    public void runOpMode() {


        //SlidePID.setTolerance(100);



        Intake = hardwareMap.get(CRServo.class, "Intake");


        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");
        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        rearRightMotor.setInverted(true);
        frontLeftMotor.setInverted(true);

        rearLeftMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        frontRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);
        rearRightMotor.setDistancePerPulse(DISTANCE_PER_PULSE);

        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();



        /*odometry = new HolonomicOdometry(
                ()->-rearLeftMotor.getDistance(),
                ()->-frontRightMotor.getDistance(),
                ()->-rearRightMotor.getDistance(),
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );*/

        odometry = new OdometryRead();

        //rightSlider = new MotorEx(hardwareMap, "rightSlider");
        //leftSlider = new MotorEx(hardwareMap, "leftSwing");

        //SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        //rightSlider.resetEncoder();
        //leftSlider.resetEncoder();
        //SlideMotion.resetEncoder();

        //leftSlider.setInverted(true);

        //odometry.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        //telemetry.addData("Robot Position at Init: ", odometry.getPose());

        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                //odometry.updatePose();

                m_Drive.driveRobotCentric(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                );


                Intake.set(1);

                //SlideMotion.set(SlidePID.calculate(SlideMotion.getCurrentPosition()) *0.2);
                //rightSlider.set(SwingSlidePID.calculate(leftSlider.getCurrentPosition()));
                //leftSlider.set(SwingSlidePID.calculate(leftSlider.getCurrentPosition()));
                //telemetry.addData("Encoder: ", SlideMotion.getCurrentPosition());

                telemetry.addData("Color: ", Sensor_Color.Color());

                telemetry.update();

                /*if(gamepad2.a){
                    SlidePID.setSetPoint(-1460);
                    //SwingSlidePID.setSetPoint(0);
                } else if (gamepad2.b){
                    SlidePID.setSetPoint(-80);
                    //SwingSlidePID.setSetPoint(500);
                } else if (gamepad2.x) {
                    //SwingSlidePID.setSetPoint(8000);
                    SlidePID.setSetPoint(-1000);
                }*/

                telemetry.addData("X:", odometry.xPos);
                telemetry.addData("Y:", odometry.yPos);
                telemetry.addData("Angulo:", odometry.zAngle);
            }
        }
    }


}
