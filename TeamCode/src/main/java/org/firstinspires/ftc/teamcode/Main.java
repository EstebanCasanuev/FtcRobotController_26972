package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

@TeleOp(name = "Teleop")
public class Main extends CommandOpMode {

    DrivetrainSubsystem m_drive;
    Drive DrivetrainCommand;

    ArmSubsystem Arm;

    Trigger Gamepad2_b = gamepad2.b;

    Robot m_robot = new MyRobot(MyRobot.OpModeType.TELEOP);

    @Override
    public void initialize() {

        m_drive = new DrivetrainSubsystem();
        DrivetrainCommand = new Drive(m_drive,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_y,
                () ->gamepad1.right_stick_x
        );

        Arm = new ArmSubsystem();
        MoveArm = new MoveArm(Arm);

        CommandScheduler.getInstance().setDefaultCommand(m_drive, DrivetrainCommand);

        CommandScheduler.getInstance().
        waitForStart();



        Gamepad2_b.toggleWhenActive(new RunCommand(() -> Arm.Setpos(()->Constants.ArmSetpoints.Deposit), Arm));


    }
}

