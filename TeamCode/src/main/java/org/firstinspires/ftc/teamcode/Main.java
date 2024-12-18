package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.*;

@Config
@TeleOp(name = "Main")
public class Main extends LinearOpMode {

    public static double P_SLIDEMOTION= 0.1;
    public static double I_SLIDEMOTION= 0.00;
    public static double D_SLIDEMOTION= 0.00;

    public static double P_SWINGMOTION= 0.1;
    public static double I_SWINGMOTION= 0.00;
    public static double D_SWINGMOTION= 0.00;

    private MotorEx frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor;
    //private MotorEx rightSlider, leftSlider;
    //private MotorEx SlideMotion;

    //PIDController SlidePID = new PIDController(P_SLIDEMOTION, I_SLIDEMOTION, D_SLIDEMOTION);
    //PIDController SwingSlidePID = new PIDController(P_SWINGMOTION, I_SWINGMOTION, D_SWINGMOTION);

    MecanumDrive m_Drive;
    @Override
    public void runOpMode() {

        //SlidePID.setTolerance(100);

        frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        rearRightMotor = new MotorEx(hardwareMap, "rearRightMotor");
        rearLeftMotor = new MotorEx(hardwareMap, "rearLeftMotor");
        m_Drive = new MecanumDrive(frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor);

        //rightSlider = new MotorEx(hardwareMap, "rightSlider");
        //leftSlider = new MotorEx(hardwareMap, "leftSwing");

        //SlideMotion = new MotorEx(hardwareMap, "SlideMotion");

        //rightSlider.resetEncoder();
        //leftSlider.resetEncoder();
        //SlideMotion.resetEncoder();

        //leftSlider.setInverted(true);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                m_Drive.driveRobotCentric(
                        gamepad1.left_stick_x,
                        0.5,
                        gamepad1.right_stick_x
                );

                /*SlideMotion.set(SlidePID.calculate(SlideMotion.getCurrentPosition()) *0.2);
                //rightSlider.set(SwingSlidePID.calculate(leftSlider.getCurrentPosition()));
                //leftSlider.set(SwingSlidePID.calculate(leftSlider.getCurrentPosition()));
                telemetry.addData("Encoder: ", SlideMotion.getCurrentPosition());
                telemetry.update();

                if(gamepad2.a){
                    SlidePID.setSetPoint(-2060);
                    //SwingSlidePID.setSetPoint(0);
                } else if (gamepad2.b){
                    SlidePID.setSetPoint(-80);
                    //SwingSlidePID.setSetPoint(500);
                } else if (gamepad2.x) {
                    //SwingSlidePID.setSetPoint(8000);
                    SlidePID.setSetPoint(-1000);
                }*/
            }
        }
    }
}
