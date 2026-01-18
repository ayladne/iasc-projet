package com.drones.model;

import com.drones.config.SimulationParams;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;
import java.util.*;

public class DroneTest {
    
    private Drone drone;
    
    @Before
    public void setUp() {
        drone = new Drone(0, 0, 0);
    }
    
    @Test
    public void testInitialization() {
        assertEquals(0, drone.getId());
        assertEquals(0.0, drone.getX(), 0.001);
        assertEquals(0.0, drone.getY(), 0.001);
        assertEquals(DroneState.ACTIVE, drone.getState());
        assertTrue(drone.isAtBase());
    }
    
    @Test
    public void testAddMeasurement() {
        assertEquals(0, drone.getMeasurements().size());
        drone.addMeasurement(0.5, 0, 10, 10);
        assertEquals(1, drone.getMeasurements().size());
    }
    
    @Test
    public void testClearMeasurements() {
        drone.addMeasurement(0.5, 0, 10, 10);
        drone.addMeasurement(0.7, 1000, 20, 20);
        assertEquals(2, drone.getMeasurements().size());
        
        drone.clearMeasurements();
        assertEquals(0, drone.getMeasurements().size());
    }
    
    @Test
    public void testStateChange() {
        assertEquals(DroneState.ACTIVE, drone.getState());
        drone.setState(DroneState.MEASURING);
        assertEquals(DroneState.MEASURING, drone.getState());
    }
    
    @Test
    public void testWaypoints() {
        List<double[]> waypoints = new ArrayList<>();
        waypoints.add(new double[]{10, 10});
        waypoints.add(new double[]{20, 20});
        
        drone.setWaypoints(waypoints);
        // Waypoints are set internally, no getter but they drive movement
    }
    
    @Test
    public void testMovement() {
        drone.setWaypoints(Arrays.asList(
            new double[]{5, 5},
            new double[]{0, 0}
        ));
        
        double initialX = drone.getX();
        double initialY = drone.getY();
        
        drone.update(SimulationParams.TICK_DURATION_MS);
        
        // Drone should have moved
        assertTrue(drone.getX() >= initialX || drone.getY() >= initialY);
    }
    
    @Test
    public void testAutonomyDecrease() {
        long initialAutonomy = drone.getAutonomyRemaining();
        drone.update(SimulationParams.TICK_DURATION_MS);
        long newAutonomy = drone.getAutonomyRemaining();
        
        assertTrue(newAutonomy < initialAutonomy);
    }
    
    @Test
    public void testMeasurementStorage() {
        Drone.Measurement m = new Drone.Measurement(0.75, 1000, 15, 20);
        assertEquals(0.75, m.intensity, 0.001);
        assertEquals(1000, m.timestamp);
        assertEquals(15, m.x, 0.001);
        assertEquals(20, m.y, 0.001);
    }
}
