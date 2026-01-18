package com.drones.control;

import com.drones.config.SimulationParams;
import com.drones.model.*;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class SimulationEngineTest {
    
    private SimulationEngine engine;
    
    @Before
    public void setUp() {
        engine = new SimulationEngine();
    }
    
    @Test
    public void testInitialization() {
        assertNotNull(engine.getEnvironment());
        assertNotNull(engine.getDrones());
        assertEquals(SimulationParams.NUM_DRONES, engine.getDrones().size());
        assertEquals(0, engine.getSimulationTime());
        assertFalse(engine.isRunning());
    }
    
    @Test
    public void testStartStop() {
        engine.start();
        assertTrue(engine.isRunning());
        engine.stop();
        assertFalse(engine.isRunning());
    }
    
    @Test
    public void testTickUpdatesTime() {
        long initialTime = engine.getSimulationTime();
        engine.start();
        engine.tick();
        long newTime = engine.getSimulationTime();
        assertEquals(initialTime + SimulationParams.TICK_DURATION_MS, newTime);
    }
    
    @Test
    public void testDronesAreInitializedAtBase() {
        for (Drone drone : engine.getDrones()) {
            assertTrue(drone.isAtBase());
            assertEquals(DroneState.ACTIVE, drone.getState());
        }
    }
    
    @Test
    public void testReset() {
        engine.start();
        engine.tick();
        engine.stop();
        engine.reset();
        
        assertEquals(0, engine.getSimulationTime());
        assertFalse(engine.isRunning());
        assertEquals(0, engine.getEventLog().size());
        
        for (Drone drone : engine.getDrones()) {
            assertTrue(drone.isAtBase());
        }
    }
    
    @Test
    public void testEventLogging() {
        int initialLogSize = engine.getEventLog().size();
        engine.logEvent("Test event");
        assertEquals(initialLogSize + 1, engine.getEventLog().size());
        assertTrue(engine.getEventLog().get(initialLogSize).contains("Test event"));
    }
    
    @Test
    public void testTrajectoryTracking() {
        assertEquals(SimulationParams.NUM_DRONES, engine.getDroneTrajectories().size());
        for (java.util.List<double[]> traj : engine.getDroneTrajectories().values()) {
            assertTrue(traj.isEmpty()); // Initially empty
        }
    }
    
    @Test
    public void testMetricsUpdate() {
        engine.start();
        for (int i = 0; i < 10; i++) {
            engine.tick();
        }
        
        SimulationEngine.SimulationMetrics metrics = engine.getMetrics();
        assertNotNull(metrics);
        assertTrue(metrics.coveragePercentage >= 0);
        assertTrue(metrics.coveragePercentage <= 100);
        assertTrue(metrics.activeDrones >= 0);
        assertTrue(metrics.rechargingDrones >= 0);
    }
}
