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
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.asSource

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

    private val modules = listOf(flModule, frModule, blModule, brModule)

    val kinematics = SwerveDriveKinematics(
            Translation2d(baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0),
            Translation2d(baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, baseLen.inMeters() / 2.0),
            Translation2d(-baseWidth.inMeters() / 2.0, -baseLen.inMeters() / 2.0)
    )

    val odometry = SwerveDriveOdometry(kinematics, Pose2d())

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
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                modules.forEachIndexed { index, module ->
                    module.output = Mk2SwerveModule.Output.Percent(states[index].speedMetersPerSecond, states[index].angle)
                }
            }
            is Output.Velocity -> {
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                modules.forEachIndexed { index, module ->
                    module.output = Mk2SwerveModule.Output.Velcity(SIUnit(states[index].speedMetersPerSecond), states[index].angle)
                }
            }
        }
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
    }

}
