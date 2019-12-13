package org.team5940.pantry.lib

interface ConcurrentlyUpdatingComponent {

    fun updateState()

    fun useState() {}
}
