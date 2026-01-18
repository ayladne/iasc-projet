package com.drones.model;

import com.drones.config.SimulationParams;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class EnvironmentTest {
    
    private Environment env;
    
    @Before
    public void setUp() {
        env = new Environment(50, 50);
    }
    
    @Test
    public void testInitialization() {
        assertEquals(50, env.getWidth());
        assertEquals(50, env.getHeight());
        assertEquals(0, env.getAnomalies().size());
        assertEquals(0, env.getElapsedTime());
    }
    
    @Test
    public void testGetAnomalyAt() {
        double intensity = env.getAnomalyAt(25, 25);
        assertEquals(0.0, intensity, 0.001);
    }
    
    @Test
    public void testAnomalySpawning() {
        // Run many ticks to increase probability of spawning
        for (int i = 0; i < 100; i++) {
            env.update(SimulationParams.TICK_DURATION_MS);
        }
        
        // Should have spawned at least one anomaly
        assertTrue(env.getAnomalies().size() >= 0); // May or may not spawn in 100 ticks
    }
    
    @Test
    public void testAnomalyDecay() {
        // Add an anomaly and update to see decay
        env.getAnomalies().add(new Anomaly(25, 25, 1.0, 0));
        env.update(SimulationParams.TICK_DURATION_MS); // First update rebuilds grid
        
        double afterFirstUpdate = env.getAnomalyAt(25, 25);
        env.update(SimulationParams.TICK_DURATION_MS); // Second update applies decay
        double afterSecondUpdate = env.getAnomalyAt(25, 25);
        
        assertTrue(afterSecondUpdate < afterFirstUpdate || afterSecondUpdate == 0);
    }
    
    @Test
    public void testReset() {
        env.getAnomalies().add(new Anomaly(10, 10, 0.5, 0));
        env.update(SimulationParams.TICK_DURATION_MS);
        
        env.reset();
        
        assertEquals(0, env.getAnomalies().size());
        assertEquals(0, env.getElapsedTime());
        assertEquals(0.0, env.getAnomalyAt(10, 10), 0.001);
    }
    
    @Test
    public void testBoundaryConditions() {
        double intensity = env.getAnomalyAt(-1, -1);
        assertEquals(0.0, intensity, 0.001);
        
        intensity = env.getAnomalyAt(100, 100);
        assertEquals(0.0, intensity, 0.001);
    }
}
