package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.utils.asSource
import wpilibj.controller.SimpleMotorFeedforward

object DriveSubsystem : FalconSubsystem() {

    private val baseWidth = 24.inches // TODO check
    private val baseLen = 24.inches // TODO check

    val gyro = AHRS(SPI.Port.kMXP).asSource()

    private val driveNativeUnitModel = SlopeNativeUnitModel(
            1.inches,
            (1.0 / (4.0 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0)).nativeUnits)

    private val flModule = Mk2SwerveModule(0, 0, 0.radians, FalconMAX(
            CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0001)

    private val frModule = Mk2SwerveModule(1, 1, 0.radians, FalconMAX(
            CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0001)

    private val blModule = Mk2SwerveModule(2, 2, 0.radians, FalconMAX(
            CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0001)

    private val brModule = Mk2SwerveModule(3, 3, 0.radians, FalconMAX(
            CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0001)

    val modules = listOf(flModule, frModule, blModule, brModule)

    val feedForward = MotorCharacterization<Meter>(
            SIUnit(0.8),
            SIUnit(0.1),
            SIUnit(0.5)
    ).apply {
        TODO("idk -- need to tune dis. Should be per module!")
    }

    val kinematics = SwerveDriveKinematics(
            Translation2d(baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0),
            Translation2d(baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0)
    )

    private val odometry = SwerveDriveOdometry(kinematics, Pose2d())

    val periodicIO = PeriodicIO()

    fun updateState() {
        modules.forEach { it.updateState() }

        // Update odometry
        val states = listOf(
                flModule.state,
                frModule.state,
                blModule.state,
                brModule.state)

        periodicIO.pose = odometry.update(gyro(), states[0], states[1], states[2], states[3])
        periodicIO.speed = kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3])
    }

    fun useState() {
        // switch over our wanted state
        // and set module positions/outputs accordingly
        when(val output = periodicIO.output) {
            is Output.Nothing -> {
                modules.forEach { it.output = Mk2SwerveModule.Output.Nothing }
            }
            is Output.Percent -> {
                // normalize wheel speeds
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0)

                modules.forEachIndexed { index, module ->
                    module.output = Mk2SwerveModule.Output.Percent(states[index].speedMetersPerSecond, states[index].angle)
                }
            }
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
                blModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                brModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is Output.TrajectoryTrackerOutput -> {
                flModule.output = output.flState
                frModule.output = output.frState
                blModule.output = output.blState
                brModule.output = output.brState
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

        class Percent(
                val chassisSpeed: ChassisSpeeds
        ) : Output()

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
