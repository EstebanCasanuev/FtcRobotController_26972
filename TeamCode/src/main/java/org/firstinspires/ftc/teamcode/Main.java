package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.*;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {

    public static double P_SLIDEMOTION= 0.00;
    public static double I_SLIDEMOTION= 0.00;
    public static double D_SLIDEMOTION= 0.00;

    public static double P_SWINGMOTION= 0.00;
    public static double I_SWINGMOTION= 0.00;
    public static double D_SWINGMOTION= 0.00;

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    private MotorEx rightSlider, leftSlider;
    private MotorEx SlideMotion;

    PIDController SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
    PIDController SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

    MecanumDrive m_Drive;
    @Override
    public void runOpMode() {

        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");
        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        rightSlider = new MotorEx(hardwareMap, "rightSlider");
        leftSlider = new MotorEx(hardwareMap, "leftSlider");

        SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        rightSlider.resetEncoder();
        leftSlider.resetEncoder();
        SlideMotion.resetEncoder();

        leftSlider.setInverted(true);

        waitForStart();
        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                m_Drive.driveRobotCentric(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                );

                SlideMotion.set(SlidePID.calculate(SlideMotion.getCurrentPosition()));
                rightSlider.set(SwingSlidePID.calculate(rightSlider.getCurrentPosition()));
                leftSlider.set(SwingSlidePID.calculate(rightSlider.getCurrentPosition()));



                if(gamepad2.a){
                    SlidePID.setSetPoint(200);
                    SwingSlidePID.setSetPoint(0);
                } else if (gamepad2.b){
                    SlidePID.setSetPoint(500);
                }
            }
        }
    }
}
