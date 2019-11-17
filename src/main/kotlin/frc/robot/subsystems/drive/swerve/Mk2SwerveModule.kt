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

        // check if we should reverse the angle
        when(val customizedOutput = customizeAngle(periodicIO.desiredOutput)) {
            is Output.Nothing -> {
                driveMotor.setNeutral()
                azimuthController.setSetpoint(0.0)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
            is Output.Percent -> {
                driveMotor.setDutyCycle(customizedOutput.percent)
                azimuthController.setSetpoint(output.angle.radians)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
            is Output.Velocity -> {
                driveMotor.setVelocity(customizedOutput.velocity, customizedOutput.arbitraryFeedForward)
                azimuthController.setSetpoint(output.angle.radians)
                azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))
            }
        }
    }

    /**
     * Decide if we should reverse the module
     * if so, reverse it
     */
    private fun customizeAngle(output: Output): Output {
        var targetAngle = output.angle
        val currentAngle = periodicIO.state.angle

        // Deltas that are greater than 90 deg or less than -90 deg can be
        // inverted so the total movement of the module
        // is less than 90 deg by inverting the wheel direction
        val delta = targetAngle - currentAngle
        if(delta.degrees > 90.0 || delta.degrees < -90.0) {
            targetAngle += 180.degrees.toRotation2d()
            output.reverse()
        }
        return output

    }

    protected class PeriodicIO {
        var distance = 0.meters
        var state = SwerveModuleState()
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output(val angle: Rotation2d) {

        abstract fun reverse(): Output

        object Nothing : Output(0.degrees.toRotation2d()) {
            override fun reverse() = this
        }

        class Percent(
                val percent: Double,
                angle: Rotation2d
        ) : Output(angle) {
            override fun reverse(): Output {
                return Percent(-percent, angle + 180.degrees.toRotation2d())
            }
        }

        class Velocity(
                val velocity: SIUnit<LinearVelocity>,
                angle: Rotation2d,
                val arbitraryFeedForward: SIUnit<Volt> = 0.volts
        ): Output(angle) {
            constructor() : this(0.meters.velocity, 0.degrees.toRotation2d(), 0.volts)
            
            override fun reverse(): Output {
                return Velocity(-velocity, angle + 180.degrees.toRotation2d(), -arbitraryFeedForward)
            }
        }
    }

}