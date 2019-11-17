package frc.robot.subsystems.drive.swerve

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Spark
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import lib.PidController
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.motors.rev.FalconMAX
import kotlin.math.PI

class Mk2SwerveModule(azimuthPAMPort: Int, azimuthAnalogPort: Int, private val offset: SIUnit<Radian>,
                      val driveMotor: FalconMAX<Meter>, angleKp: Double, angleKd: Double) {

    private val stateMutex = Object()
    protected val periodicIO = PeriodicIO()
        get() = synchronized(stateMutex) { field }

    val distance get() = periodicIO.distance
    val state get() = periodicIO.state
    var output: Output
        get() = periodicIO.desiredOutput
        set(value) { periodicIO.desiredOutput = value }

    private val azimuthMotor = Spark(azimuthPAMPort)
    private val azimuthController = PidController(angleKp, angleKd).apply {
        setInputRange(0.0, 2.0 * PI)
        setContinuous(true)
        setOutputRange(-0.5, 0.5)
    }

    private val analogInput = AnalogInput(azimuthAnalogPort)
    val azimuthAngle =
            { ((1.0 - analogInput.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + offset).toRotation2d() }

    init {
        driveMotor.canSparkMax.apply {
            setSmartCurrentLimit(60);
            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 500);
            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 3);
        }
    }

    fun updateState() {
        periodicIO.distance = driveMotor.encoder.position
        periodicIO.state = SwerveModuleState(
                driveMotor.encoder.velocity.value,
                azimuthAngle())
    }

    fun useState() {
        when(val output = periodicIO.desiredOutput) {
            is Output.Nothing -> {
                driveMotor.setNeutral()
                azimuthController.setSetpoint(0.0)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
            is Output.Percent -> {
                driveMotor.setDutyCycle(output.percent)
                azimuthController.setSetpoint(output.angle.radians)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
            is Output.Velocity -> {
                driveMotor.setVelocity(output.velocity, output.arbitraryFeedForward)
                azimuthController.setSetpoint(output.angle.radians)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
        }
    }

    protected class PeriodicIO {
        var distance = 0.meters
        var state = SwerveModuleState()
        var desiredOutput: Output = Output.Nothing
    }

    public sealed class Output {
        object Nothing : Output()

        class Percent(
                val percent: Double,
                val angle: Rotation2d
        ) : Output()

        class Velocity(
                val velocity: SIUnit<LinearVelocity>,
                val angle: Rotation2d,
                val arbitraryFeedForward: SIUnit<Volt> = 0.volts
        ): Output() {
            constructor() : this(0.meters.velocity, 0.degrees.toRotation2d(), 0.volts)
        }
    }

}