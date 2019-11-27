package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import org.ghrobotics.lib.wrappers.hid.*
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.autonomous.paths.TrajectoryWaypoints
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveTrajectoryFollowerCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryGenerator
import org.ghrobotics.lib.mathematics.units.derived.acceleration
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.feet

object Controls {

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {
        button(kX).changeOn(InstantCommand(Runnable { println("Hullo!") }))

        val trajectory = FalconTrajectoryGenerator.generateTrajectory(
                listOf(TrajectoryWaypoints.kLoadingStation, TrajectoryWaypoints.kCargoShipS3.mirror()),
                FalconTrajectoryConfig(10.feet.velocity, 10.feet.acceleration)
        )

        val command = SwerveTrajectoryFollowerCommand { trajectory }

        button(kB).changeOn(
                command
        )
    }

    val operatorWPIJoystick = XboxController(1)
    val operatorFalconXbox = operatorWPIJoystick.mapControls {
    }

    fun update() {
        driverFalconXbox.update()
//        operatorFalconHID.update()
        operatorFalconXbox.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
