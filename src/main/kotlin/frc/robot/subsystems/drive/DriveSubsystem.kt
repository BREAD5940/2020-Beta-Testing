package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import frc.robot.Constants
import frc.robot.Constants.baseLen
import frc.robot.Constants.baseWidth
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.asSource
import org.ghrobotics.lib.utils.launchFrequency

object DriveSubsystem : FalconSubsystem() {

//    val gyro = AHRS(SPI.Port.kMXP).asSource()


    private val driveNativeUnitModel = SlopeNativeUnitModel(
            1.inches,
            (1.0 / (4.0 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0)).nativeUnits)

    private val flModule = Mk2SwerveModule()//0, 0, 0.radians, FalconMAX(
//            CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
//            0.5, 0.0001)

    private val frModule = Mk2SwerveModule()//1, 1, 0.radians, FalconMAX(
//            CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
//            0.5, 0.0001)

    private val blModule = Mk2SwerveModule()//2, 2, 0.radians, FalconMAX(
//            CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
//            0.5, 0.0001)

    private val brModule = Mk2SwerveModule()//3, 3, 0.radians, FalconMAX(
//            CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
//            0.5, 0.0001)

    val modules = listOf(flModule, frModule, blModule, brModule)

    val feedForward = MotorCharacterization<Meter>(
            SIUnit(0.8),
            SIUnit(0.1),
            SIUnit(0.5)
    ).apply {
        TODO("idk -- need to tune dis. Should be per module!")
    }

    val kinematics = SwerveDriveKinematics(
            Constants.kModulePositions[0],
            Constants.kModulePositions[1],
            Constants.kModulePositions[2],
            Constants.kModulePositions[3]
    )

    val robotPosition get() = periodicIO.pose

    internal val odometry = SwerveDriveOdometry(kinematics, Rotation2d())

    private val stateLock = Object()
    val periodicIO = PeriodicIO()
        get() = synchronized(stateLock) { field }

    override fun lateInit() {

        // update localization f a s t
        this.kinematicsUpdateJob = GlobalScope.launchFrequency(200) {
            updateState()
            useState()
        }

        lastUpdateTime = Timer.getFPGATimestamp()
    }

    override fun periodic() {
        if (!kinematicsUpdateJob.isActive) kinematicsUpdateJob.start()

        LiveDashboard.robotHeading = robotPosition.rotation.radians
        LiveDashboard.robotX = robotPosition.translation.x_u.inFeet()
        LiveDashboard.robotY = robotPosition.translation.y_u.inFeet()

    }

    private var kinematicsUpdateJob: Job
    private var lastUpdateTime = 0.0
    private fun updateState() {
        modules.forEach { it.updateState() }

        // Update odometry
        val states = listOf(
                flModule.state,
                frModule.state,
                brModule.state,
                blModule.state)

        val now = Timer.getFPGATimestamp()
        val dt = now - lastUpdateTime



        periodicIO.speed = kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3])
        periodicIO.pose = odometry.update(periodicIO.pose.rotation, states[0], states[1], states[2], states[3])
        periodicIO.pose = Pose2d(
                periodicIO.pose.translation,
                periodicIO.pose.rotation + (periodicIO.speed.omegaRadiansPerSecond * dt).radians.toRotation2d()
        )

        lastUpdateTime = now
    }

    fun useState() {
        // switch over our wanted state
        // and set module positions/outputs accordingly
        when (val output = periodicIO.output) {
            is Output.Nothing -> {
                modules.forEach { it.output = Mk2SwerveModule.Output.Nothing }
            }
//            is Output.Percent -> {
//                // normalize wheel speeds
//                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
//                SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0)
//
//                modules.forEachIndexed { index, module ->
//                    module.output = Mk2SwerveModule.Output.Percent(states[index].speedMetersPerSecond, states[index].angle)
//                }
//            }
            is Output.Velocity -> {
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                modules.forEachIndexed { index, module ->
                    module.output = Mk2SwerveModule.Output.Velocity(SIUnit(states[index].speedMetersPerSecond), states[index].angle)
                }
            }
            is Output.KinematicsVelocity -> {
                flModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[0].speedMetersPerSecond),
                        output.speeds[0].angle
                )
                frModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[1].speedMetersPerSecond),
                        output.speeds[1].angle
                )
                brModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                blModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is Output.TrajectoryTrackerOutput -> {
                flModule.output = output.flState
                frModule.output = output.frState
                brModule.output = output.brState
                blModule.output = output.blState
            }
        }

        modules.forEach { it.useState() }
    }

    class PeriodicIO {
        var pose = Pose2d()
        var speed = ChassisSpeeds()
        var output: Output = Output.Nothing
    }

    sealed class Output {
        object Nothing : Output()

//        class Percent(
//            val chassisSpeed: ChassisSpeeds
//        ) : Output()

        class Velocity(
            val chassisSpeed: ChassisSpeeds
        ) : Output()

        class KinematicsVelocity(
            val speeds: List<SwerveModuleState>
        ) : Output()

        class TrajectoryTrackerOutput(
            val flState: Mk2SwerveModule.Output.Velocity,
            val frState: Mk2SwerveModule.Output.Velocity,
            val blState: Mk2SwerveModule.Output.Velocity,
            val brState: Mk2SwerveModule.Output.Velocity
        ) : Output() {
            constructor() : this (
                    Mk2SwerveModule.Output.Velocity(),
                    Mk2SwerveModule.Output.Velocity(),
                    Mk2SwerveModule.Output.Velocity(),
                    Mk2SwerveModule.Output.Velocity()
            )
        }
    }
}
