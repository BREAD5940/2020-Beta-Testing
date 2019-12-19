package frc.robot.subsystems.superstructure.controller

import com.google.gson.Gson
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N2
import frc.robot.Robot
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Superstructure
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import org.ejml.simple.SimpleMatrix
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.utils.launchFrequency
import org.team5940.pantry.lib.WantedState
import java.text.DecimalFormat
import kotlin.concurrent.timer

class ElevatorSSTest(val targetHeight: SIUnit<Meter>): FalconCommand(Superstructure, Elevator) {

    private val constraints = TrapezoidProfile.Constraints(24.inches.inMeters(), 400.inches.inMeters())

    override fun isFinished(): Boolean {
        return false
    }

//    val notifier = falconnotifier(this::execute0)
    lateinit var job: Job

    override fun initialize() {
        Elevator.motor.encoder.resetPosition(0.meters)
//        list = ""
//        list = "time, xHat(0), xHat(1), position(encoder), velocity(encoder)"

        ElevatorController.reset()

//        notifier.startPeriodic(0.005)
        job = GlobalScope.launchFrequency(200) { execute0() }
        job.start()

        ElevatorController.enable()

        list = ArrayList<String>()
        list.add("time, xHat(0), xHat(1), voltage, position(encoder), velocity(encoder), refPos, refVel")
    }

    private fun execute0() {

        if(!Robot.isEnabled) {
            return
        }

        // let's take the elevator up of it's down and down if it's up
//        val targetHeight = 20.inches

//        ElevatorController.enable()


        val state = Elevator.currentState

        val profile = TrapezoidProfile(
                constraints, TrapezoidProfile.State(targetHeight.inMeters(), 0.0),
                TrapezoidProfile.State(state.position.value, state.velocity.value)
        )

        val target = profile.calculate(0.0050)

        System.out.println("current ${state.position.value} target ${target.position}")

//        ElevatorController.nextR = MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(target.position, target.velocity)
        ElevatorController.nextR = MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(targetHeight.inMeters(), 0.0)

        val y = ElevatorController.plant.calculateY(
                MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(state.position.value, state.velocity.value),
                ElevatorController.u)

        ElevatorController.correct(y)

//        ElevatorController.correct(MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(state.position.value, state.velocity.value))
        ElevatorController.predict(0.020)

        var voltage = ElevatorController.getU(0)

        println("error ${ElevatorController.error.get(0, 0).meters.inInches().toInt()} voltage: ${df.format(voltage)}")

        // states, inputs, outputs
        var data = "${Timer.getFPGATimestamp()}, ${ElevatorController.observer.getXhat(0)}, ${ElevatorController.observer.getXhat(1)}," +
                "$voltage, ${state.position.inMeters()}, ${state.velocity.value}, ${target.position}, ${target.velocity}\n"

//        println(data)

        list.add(data)

        SmartDashboard.putString("elevatorJson", gson.toJson(list))

//        SmartDashboard.putStringArray("elevatorData", list.toTypedArray())

//        voltage = voltage.coerceIn(-4.0, 4.0)
        Elevator.wantsLowGear = false
        Elevator.wantedState = WantedState.Voltage(voltage.volts)
    }

    override fun end(interrupted: Boolean) {
        Elevator.wantedState = WantedState.Nothing
        ElevatorController.disable()
        job.cancel()
    }

    private var gson = Gson()
    private var list = ArrayList<String>()
    val df = DecimalFormat("##.#")

}