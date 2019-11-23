package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryGenerator
import org.ghrobotics.lib.mathematics.units.derived.acceleration
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.feet

object TestTrajectory {
   var trajectory = FalconTrajectoryGenerator.generateTrajectory(listOf(org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d(10.feet,10.feet,0.degrees), Pose2d(15.feet,10.feet,0.degrees)), FalconTrajectoryConfig(5.feet.velocity, 6.feet.acceleration))
}