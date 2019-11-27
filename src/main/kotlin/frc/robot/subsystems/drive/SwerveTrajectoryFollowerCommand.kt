package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import lib.PidController
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.utils.Source

class SwerveTrajectoryFollowerCommand(val trajectorySupplier: Source<Trajectory>) : FalconCommand(DriveSubsystem) {

    private var prevStates = listOf<SwerveModuleState>()
    lateinit var trajectory: Trajectory
    private val timer = Timer()
    var prevTime = 0.0
    var lastOutput = DriveSubsystem.Output.TrajectoryTrackerOutput()

    private val forwardController = PidController(2.0, 0.0) // x meters per second per meter of error
    private val strafeController = PidController(2.0, 0.0)
    private val rotationController = PidController(0.5, 0.0) // rad per sec per radian of error

    override fun initialize() {
        trajectory = trajectorySupplier()
        timer.reset()
        timer.start()
        prevTime = 0.0
        FalconDashboard.isFollowingPath = false
        FalconDashboard.isFollowingPath = true
        prevStates = listOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
    }

    override fun end(interrupted: Boolean) {
        FalconDashboard.isFollowingPath = true
    }

    override fun execute() {
        val time = timer.get()
        val dt = time - prevTime

        // Our wanted position on the field
        val state = trajectory.sample(time)
        val velocity = state.poseMeters.rotation.toTranslation2d() * state.velocityMetersPerSecond
        val currentPose = DriveSubsystem.periodicIO.pose

        // P loop to convert delta in X and Y to velocity outputs, and rotation to rotations speed
        forwardController.setSetpoint(state.poseMeters.translation.x)
        strafeController.setSetpoint(state.poseMeters.translation.y)
        rotationController.setSetpoint(state.poseMeters.rotation.radians)

        // place the output in the robot frame of reference
        // and add the trajectory velocity to it as a feedforward
        val feedbackOutput = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardController.calculate(currentPose.translation.x, dt) + velocity.x,
                strafeController.calculate(currentPose.translation.y, dt) + velocity.y,
                rotationController.calculate(currentPose.rotation.radians, dt),
                currentPose.rotation
        )

        // convert from chassis speeds (PID plus trajectory speeds) to states
        val states = DriveSubsystem.kinematics.toSwerveModuleStates(feedbackOutput).toList()

        // Calculate feedforwards for each module based on it's acceleration
        val ff = DriveSubsystem.feedForward
        val outputs = arrayListOf<Mk2SwerveModule.Output.Velocity>() // Temp array
        states.forEachIndexed { index, _ ->
            val acceleration =
                (states[index].speedMetersPerSecond - prevStates[index].speedMetersPerSecond) / dt
            val moduleVelocity = states[index].speedMetersPerSecond

            val ffVoltage = ff.getVoltage(SIUnit(moduleVelocity), SIUnit(acceleration))

            outputs[index] = Mk2SwerveModule.Output.Velocity(
                    SIUnit(moduleVelocity),
                    states[index].angle,
                    ffVoltage
            )
        }

        DriveSubsystem.periodicIO.output = DriveSubsystem.Output.TrajectoryTrackerOutput(
                outputs[0], outputs[1], outputs[2], outputs[3]
        )

        // output to smartdashboard
        FalconDashboard.pathX = state.poseMeters.translation.x_u.inFeet()
        FalconDashboard.pathY = state.poseMeters.translation.y_u.inFeet()
        FalconDashboard.pathHeading = state.poseMeters.rotation.radians

        prevTime = time
        prevStates = states
    }
}

private fun Rotation2d.toTranslation2d() = Translation2d(this.cos, this.sin)
