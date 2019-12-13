package org.team5940.pantry.lib

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.subsystems.drive.DriveSubsystem
//import frc.robot.subsystems.sensors.LimeLight
import frc.robot.subsystems.superstructure.* // ktlint-disable no-wildcard-imports
//import frc.robot.vision.LimeLightManager
import kotlinx.coroutines.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.utils.loopFrequency
import org.ghrobotics.lib.wrappers.FalconTimedRobot

abstract class FishyRobot : FalconTimedRobot() {

    val isEnabled
            get() = wrappedValue.isEnabled

    val isAuto get() = wrappedValue.isAutonomous

    private fun periodicUpdate() {

        SmartDashboard.putNumber("lastTry", Timer.getFPGATimestamp())

//        DriveSubsystem.updateState()
//        DriveSubsystem.useState()
//        Superstructure.updateState()
//        Superstructure.useState()

//        LimeLight.update()
//        LimeLightManager.periodic()
    }

//    private val job = arrayListOf<Job>()
var updateJob: Job? = null
    var ntJob: Job? = null

//    var lastRobotMode = Mode.DISABLED
//        private set

    override fun robotInit() {

        updateJob = (updateScope.launch {
            loopFrequency(75 /* hertz */) {
                try { periodicUpdate() } catch (ignored: Exception) {}
            }
        })

        ntJob = (updateScope.launch {
            loopFrequency(2) {
//                SmartDashboard.putString("Joint states", Superstructure.currentState.asString())
                val elevatorTarget = Elevator.wantedState as? WantedState.Position<*>
                        ?: WantedState.Position(0.inch)
                println("Elevator target ${elevatorTarget.targetPosition.value.meters.inch} position ${Elevator.currentState.position.inch}")

//                val proxTarget = Proximal.wantedState as? WantedState.Position<*>
//                        ?: WantedState.Position(0.degree)
//                println("Elevator target ${proxTarget.targetPosition.value.radian.degree} position ${Proximal.currentState.position.degree}")
//
//                val wristTarget = Wrist.wantedState as? WantedState.Position<*>
//                        ?: WantedState.Position(0.degree)
//                println("Elevator target ${wristTarget.targetPosition.value.radian.degree} position ${Wrist.currentState.position.degree}")
            }
        })

        CommandScheduler.getInstance().onCommandInitialize { command -> println("[CommandScheduler] Command ${command.name} initialized!") }
//        CommandScheduler.getInstance().onCommandExecute { command -> println("[CommandScheduler] Command ${command.name} execute!") }
        CommandScheduler.getInstance().onCommandInterrupt { command -> println("[CommandScheduler] Command ${command.name} interrupted!") }
        CommandScheduler.getInstance().onCommandFinish { command -> println("[CommandScheduler] Command ${command.name} finished!!") }

        super.robotInit()
    }

    override fun disabledInit() {
        super.disabledInit()
    }

    override fun autonomousInit() {
        super.autonomousInit()
    }

    override fun teleopInit() {
        super.teleopInit()
    }

    override fun robotPeriodic() {
        updatableSubsystems.forEach { it.update() }

        val job = this.updateJob
        if (job != null) {
            if (!job.isActive) job.start()
        }

        val job2 = this.ntJob
        if (job2 != null) {
            if (!job2.isActive) job2.start()
        }


//        runBlocking { periodicUpdate() }
        super.robotPeriodic()
    }

    private val updatableSubsystems = arrayListOf<Updatable>()

    operator fun Updatable.unaryPlus() {
        updatableSubsystems.add(this)
    }

    companion object {
        @Suppress("EXPERIMENTAL_API_USAGE")
        val updateScope = CoroutineScope(newFixedThreadPoolContext(1, "SubsystemUpdate"))
    }
}

interface Updatable { fun update() }
