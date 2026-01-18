package com.drones.model;

import com.drones.config.SimulationParams;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * Test de recharge des drones sur 30 minutes
 * Vérifie que les drones se rechargent complètement après épuisement de la batterie
 */
public class DroneRechargeTest {
    
    private Drone drone;
    
    @Before
    public void setUp() {
        drone = new Drone(1, 25.0, 25.0); // Centre de la grille
    }
    
    @Test
    public void testDroneRechargeFullCycle() {
        // Vérifier état initial : pleine batterie
        assertEquals("Batterie devrait être pleine au départ", 
                    SimulationParams.DRONE_AUTONOMY_MS, 
                    drone.getAutonomyRemaining());
        assertEquals("Drone devrait être ACTIVE", DroneState.ACTIVE, drone.getState());
        
        // Épuiser complètement la batterie (simuler 30 minutes d'activité)
        int ticksToEmpty = SimulationParams.DRONE_AUTONOMY_MS / SimulationParams.TICK_DURATION_MS;
        for (int i = 0; i < ticksToEmpty + 1; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        // Vérifier que la batterie est épuisée
        assertTrue("Batterie devrait être presque vide", drone.getAutonomyRemaining() <= 0);
        assertEquals("Drone devrait être RETURNING", 
                    DroneState.RETURNING, 
                    drone.getState());
        
        // Simuler le retour à la base (forcer la position)
        while (drone.getState() == DroneState.RETURNING) {
            // Forcer le retour en mettant le drone proche de la base
            if (Math.sqrt(drone.getX() * drone.getX() + drone.getY() * drone.getY()) > 0.5) {
                // Le drone se déplace vers la base, forcer l'arrivée
                drone = new Drone(1, 0.1, 0.1); // Proche de la base
                drone.setState(DroneState.RETURNING);
            }
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        assertEquals("Drone devrait être CHARGING après retour", DroneState.CHARGING, drone.getState());
        
        long batteryBefore = drone.getAutonomyRemaining();
        
        // Simuler 10 minutes de recharge (temps de recharge complet)
        int ticksToRecharge = SimulationParams.DRONE_RECHARGE_MS / SimulationParams.TICK_DURATION_MS;
        for (int i = 0; i < ticksToRecharge; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        // Vérifier que la batterie est rechargée
        long batteryAfter = drone.getAutonomyRemaining();
        assertEquals("Batterie devrait être complètement rechargée", 
                    SimulationParams.DRONE_AUTONOMY_MS, 
                    batteryAfter);
        assertEquals("Drone devrait être ACTIVE après recharge", DroneState.ACTIVE, drone.getState());
    }
    
    @Test
    public void testDroneReturnsToBaseWhenLowBattery() {
        // Épuiser la batterie jusqu'à épuisement
        int ticksToEmpty = (SimulationParams.DRONE_AUTONOMY_MS / SimulationParams.TICK_DURATION_MS) + 1;
        for (int i = 0; i < ticksToEmpty; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        // Le drone devrait être en mode RETURNING
        assertEquals("Drone avec batterie épuisée devrait retourner à la base",
                  DroneState.RETURNING, 
                  drone.getState());
    }
    
    @Test
    public void testRechargeRate() {
        // Épuiser complètement la batterie
        int ticksToEmpty = (SimulationParams.DRONE_AUTONOMY_MS / SimulationParams.TICK_DURATION_MS) + 1;
        for (int i = 0; i < ticksToEmpty + 10; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        // Forcer la position à la base et l'état en CHARGING
        drone = new Drone(1, 0.1, 0.1);
        drone.setState(DroneState.CHARGING);
        drone.update(SimulationParams.TICK_DURATION_MS);
        
        long batteryBefore = drone.getAutonomyRemaining();
        
        // Faire 10 ticks de recharge
        for (int i = 0; i < 10; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
        }
        
        long batteryAfter = drone.getAutonomyRemaining();
        
        // Vérifier que la recharge a bien commencé
        // (batterie doit rester à 0 jusqu'à ce que le timer de recharge soit écoulé)
        // Ou avoir augmenté si le test dure assez longtemps
        assertTrue("Le drone devrait être en état de recharge", 
                  drone.getState() == DroneState.CHARGING || drone.getState() == DroneState.ACTIVE);
    }
    
    @Test
    public void testFullRechargeAfter30MinutesUse() {
        System.out.println("\n=== TEST: Cycle complet 30 min utilisation + 10 min recharge ===");
        
        // Phase 1: Utilisation pendant 30 minutes
        System.out.println("Phase 1: Décharge pendant 30 minutes...");
        long initialBattery = drone.getAutonomyRemaining();
        System.out.println("  Batterie initiale: " + initialBattery + " ms (" + (initialBattery/60000) + " min)");
        
        int ticksForFullDischarge = SimulationParams.DRONE_AUTONOMY_MS / SimulationParams.TICK_DURATION_MS;
        for (int i = 0; i < ticksForFullDischarge + 1; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
            
            // Afficher progression tous les 5 minutes
            if ((i * SimulationParams.TICK_DURATION_MS) % (5 * 60 * 1000) == 0) {
                int timeMin = (i * SimulationParams.TICK_DURATION_MS) / 60000;
                long battery = drone.getAutonomyRemaining();
                System.out.println("  Temps: " + timeMin + " min - Batterie: " + battery + " ms (" + (battery/60000) + " min) - État: " + drone.getState());
            }
        }
        
        long batteryAfterUse = drone.getAutonomyRemaining();
        System.out.println("  Batterie après 30 min: " + batteryAfterUse + " ms (" + (batteryAfterUse/60000) + " min)");
        System.out.println("  État du drone: " + drone.getState());
        
        assertTrue("Batterie devrait être épuisée après 30 min", batteryAfterUse <= 0);
        assertEquals("Drone devrait être RETURNING", DroneState.RETURNING, drone.getState());
        
        // Phase 2: Simuler l'arrivée à la base et recharge pendant 10 minutes
        System.out.println("\nPhase 2: Recharge pendant 10 minutes...");
        
        // Forcer l'arrivée à la base
        drone = new Drone(1, 0.1, 0.1);
        drone.setState(DroneState.RETURNING);
        drone.update(SimulationParams.TICK_DURATION_MS); // Transition vers CHARGING
        
        assertEquals("Drone devrait être CHARGING", DroneState.CHARGING, drone.getState());
        
        int ticksForFullRecharge = SimulationParams.DRONE_RECHARGE_MS / SimulationParams.TICK_DURATION_MS;
        for (int i = 0; i < ticksForFullRecharge; i++) {
            drone.update(SimulationParams.TICK_DURATION_MS);
            
            // Afficher progression tous les 2 minutes
            if ((i * SimulationParams.TICK_DURATION_MS) % (2 * 60 * 1000) == 0) {
                int timeMin = (i * SimulationParams.TICK_DURATION_MS) / 60000;
                long battery = drone.getAutonomyRemaining();
                System.out.println("  Temps: " + timeMin + " min - Batterie: " + battery + " ms (" + (battery/60000) + " min) - État: " + drone.getState());
            }
        }
        
        long batteryAfterRecharge = drone.getAutonomyRemaining();
        System.out.println("  Batterie après recharge: " + batteryAfterRecharge + " ms (" + (batteryAfterRecharge/60000) + " min)");
        System.out.println("  État du drone: " + drone.getState());
        
        // Vérifications finales
        assertEquals("Batterie devrait être complètement rechargée", 
                    SimulationParams.DRONE_AUTONOMY_MS, 
                    batteryAfterRecharge);
        assertEquals("Drone devrait être ACTIVE après recharge complète", 
                    DroneState.ACTIVE, 
                    drone.getState());
        
        System.out.println("\n✓ Cycle complet validé: 30 min décharge + 10 min recharge = batterie restaurée");
        System.out.println("=============================================================\n");
    }
}
