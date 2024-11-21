package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class Trajectory {
    public void generateTrayectory(){
        Pose2d sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
                Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
                Rotation2d.fromDegrees(-160));

        ArrayList interiorWaypoints = new ArrayList();
        interiorWaypoints.add(0, new Translation2d(, ));
        interiorWaypoints.add(1, new Translation2d(, ));



        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);

        TrajectoryGenerator trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }
    }
}
