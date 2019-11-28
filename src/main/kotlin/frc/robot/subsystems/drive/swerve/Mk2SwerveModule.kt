package frc.robot.subsystems.drive.swerve

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Spark
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI
import lib.PidController
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.motors.rev.FalconMAX

class Mk2SwerveModule(val name: String) {

    private val stateMutex = Object()
    protected val periodicIO = PeriodicIO()
        get() = synchronized(stateMutex) { field }

    val state get() = periodicIO.state
    var output: Output.Velocity
        get() = periodicIO.desiredOutput
        set(value) { periodicIO.desiredOutput = value }

    fun updateState() {
        periodicIO.state = SwerveModuleState(
                periodicIO.desiredOutput.velocity.value,
                periodicIO.desiredOutput.angle)
    }

    fun useState() {

        val customizedOutput = customizeAngle(periodicIO.desiredOutput)
//        azimuthController.setSetpoint(customizedOutput.angle.radians)
//        azimuthMotor.set(azimuthController.calculate(periodicIO.state.angle.radians, 0.020))

        // check if we should reverse the angle
        when (customizedOutput) {
            is Output.Nothing -> {
//                driveMotor.setNeutral()
            }
//            is Output.Percent -> {
////                driveMotor.setDutyCycle(customizedOutput.percent)
//            }
            is Output.Velocity -> {
//                driveMotor.setVelocity(customizedOutput.velocity, customizedOutput.arbitraryFeedForward)
                ntVelocityEntry.setDouble(customizedOutput.velocity.inFeetPerSecond())
            }
        }

        ntAngleEntry.setDouble(customizedOutput.angle.degrees)

    }

    /**
     * Decide if we should reverse the module
     * if so, reverse it
     */
    private fun customizeAngle(output: Output): Output {
        var targetAngle = output.angle
        val currentAngle = periodicIO.state.angle

        // The delta should already be [-pi, pi]
        // Deltas that are greater than 90 deg or less than -90 deg can be
        // inverted so the total movement of the module
        // is less than 90 deg by inverting the wheel direction
        val delta = targetAngle - currentAngle
        if(name == "fl" && delta.degrees > 1e-2) println("delta ${delta.degrees}")
        if (delta.degrees > 90.0 || delta.degrees < -90.0) {
            output.reverse()
            if(name == "fl") println(" | reversing fl!")
        }
        return output
    }

    protected class PeriodicIO {
        // the current state
        var state = SwerveModuleState()

        // the desired state
        var desiredOutput: Output.Velocity = Output.Nothing
    }

    sealed class Output(val angle: Rotation2d) {

        abstract fun reverse(): Output

        object Nothing : Velocity(SIUnit(0.0), 0.degrees.toRotation2d()) {
            override fun reverse() = this
        }

        open class Velocity(
            val velocity: SIUnit<LinearVelocity>,
            angle: Rotation2d,
            val arbitraryFeedForward: SIUnit<Volt> = 0.volts
        ) : Output(angle) {
            constructor() : this(0.meters.velocity, 0.degrees.toRotation2d(), 0.volts)

            override fun reverse(): Output {
                return Velocity(-velocity, angle + 180.degrees.toRotation2d(), -arbitraryFeedForward)
            }
        }
    }

    val ntAngleEntry = SmartDashboard.getEntry("${name}_angle")
    val ntVelocityEntry = SmartDashboard.getEntry("${name}_velocity")
//    suspend fun outputToSmartDashboard() {
//        ntAngleEntry.setDouble()
//    }
}
