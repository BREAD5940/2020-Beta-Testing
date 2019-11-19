package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import frc.robot.subsystems.drive.swerve.SwerveTrajectoryController
import lib.PidController
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.utils.Source

class SwerveTrajectoryFollowerCommand(val trajectorySupplier: Source<Trajectory>) : FalconCommand(DriveSubsystem) {

    private var prevStates = listOf<SwerveModuleState>()
    lateinit var trajectory: Trajectory
    private val timer = Timer()

    private val controller = SwerveTrajectoryController(DriveSubsystem.kinematics, DriveSubsystem.feedForward)

    override fun initialize() {
        trajectory = trajectorySupplier()
        timer.reset()
        timer.start()
        prevStates = listOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
    }

    override fun execute() {

        // update the controller
        val time = timer.get()
        val state = trajectory.sample(time)
        DriveSubsystem.periodicIO.output = controller.calculate(time, state, DriveSubsystem.periodicIO.pose)

    }
}

fun Rotation2d.toTranslation2d() = Translation2d(this.cos, this.sin)
