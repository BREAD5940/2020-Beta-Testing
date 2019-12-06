package frc.robot.subsystems.drive.swerve

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.robot.subsystems.drive.toTranslation2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.physics.MotorCharacterization

class SwerveTrajectoryController(
    private val kinematics: SwerveDriveKinematics,
    private val feedforward: MotorCharacterization<Meter>
) {

    private var lastTime = -1.0
    private var prevState = listOf(
            SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())

    private val forwardController = PIDController(1.0, 0.0, 0.0) // x meters per second per meter of error
    private val strafeController = PIDController(1.0, 0.0, 0.0)
    private val rotationController = PIDController(0.5, 0.0, 0.0) // rad per sec per radian of error

    fun calculate(
        time: Double,
        state: Trajectory.State,
        targetHeading: Rotation2d,
        currentPose: Pose2d
    ): SwerveDriveOutput.TrajectoryTrackerOutput {

        // dt
        if (lastTime < 0.0) lastTime = time
        val dt = time - lastTime

        // Our wanted position on the field
        val velocity = state.poseMeters.rotation.toTranslation2d() * state.velocityMetersPerSecond

        // P loop to convert delta in X and Y to velocity outputs, and rotation to rotations speed
        forwardController.setSetpoint(state.poseMeters.translation.x)
        strafeController.setSetpoint(state.poseMeters.translation.y)
        rotationController.setSetpoint(targetHeading.radians)

        // place the output in the robot frame of reference
        // and add the trajectory velocity to it as a feedforward
        val feedbackOutput = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardController.calculate(currentPose.translation.x, dt) + velocity.x,
                strafeController.calculate(currentPose.translation.y, dt) + velocity.y,
                rotationController.calculate(currentPose.rotation.radians, dt),
                currentPose.rotation
        )

        // convert from chassis speeds (PID plus trajectory speeds) to states
        val states = kinematics.toSwerveModuleStates(feedbackOutput).toList()

        // Calculate feedforwards for each module based on it's acceleration
        val outputs = arrayListOf<Mk2SwerveModule.Output.Velocity>() // Temp array
        states.forEachIndexed { index, _ ->
            val acceleration =
                    (states[index].speedMetersPerSecond - prevState[index].speedMetersPerSecond) / dt
            val moduleVelocity = states[index].speedMetersPerSecond

            val ffVoltage = feedforward.getVoltage(SIUnit(moduleVelocity), SIUnit(acceleration))

            outputs.add(index, Mk2SwerveModule.Output.Velocity(
                    SIUnit(moduleVelocity),
                    states[index].angle,
                    ffVoltage
            ))
        }

        prevState = states

        return SwerveDriveOutput.TrajectoryTrackerOutput(
                outputs[0], outputs[1], outputs[2], outputs[3]
        )
    }
}
