package com.drones.control;
import com.drones.config.SimulationParams;
import com.drones.model.Drone;
import com.drones.model.DroneState;
import com.drones.model.Environment;
import java.util.*;
public class Coordinator {
    private Set<String> visitedCells; // cellules déjà explorées (x,y)
    private Map<Integer, List<double[]>> droneWaypoints; // Points de passage en cache par drone
    
    public Coordinator() {
        visitedCells = new HashSet<>();
        droneWaypoints = new HashMap<>();
    }
    
    // Générer le plan de couverture initial (balayage raster)
    public Map<Integer, List<double[]>> generateCoveragePlan(int numDrones) {
        Map<Integer, List<double[]>> plan = new HashMap<>();
        
        int dronesPerRow = (int) Math.ceil(Math.sqrt(numDrones));
        int cellsPerDrone = SimulationParams.GRID_WIDTH / dronesPerRow;
        
        for (int i = 0; i < numDrones; i++) {
            List<double[]> waypoints = new ArrayList<>();
            
            int row = i / dronesPerRow;
            int col = i % dronesPerRow;
            
            int startX = col * cellsPerDrone;
            int startY = row * cellsPerDrone;
            int endX = Math.min((col + 1) * cellsPerDrone, SimulationParams.GRID_WIDTH - 1);
            int endY = Math.min((row + 1) * cellsPerDrone, SimulationParams.GRID_HEIGHT - 1);
            
            // Motif de balayage raster au sein de la région
            for (int y = startY; y <= endY; y++) {
                if ((y - startY) % 2 == 0) {
                    for (int x = startX; x <= endX; x++) {
                        waypoints.add(new double[]{x, y});
                    }
                } else {
                    for (int x = endX; x >= startX; x--) {
                        waypoints.add(new double[]{x, y});
                    }
                }
            }
            
            // Retour à la base
            waypoints.add(new double[]{0, 0});
            
            plan.put(i, waypoints);
        }
        
        return plan;
    }
    
    // Réaffectation adaptive: si une anomalie élevée est détectée, affecter les drones proches
    public void adaptiveRetasking(List<Drone> drones, Environment env) {
        double[][] grid = env.getAnomalyIntensity();
        
        // Trouver les points chauds (cellules avec anomalie élevée)
        List<int[]> hotspots = new ArrayList<>();
        for (int y = 0; y < env.getHeight(); y++) {
            for (int x = 0; x < env.getWidth(); x++) {
                if (grid[y][x] > 0.7) { // Seuil d'anomalie élevée
                    hotspots.add(new int[]{x, y});
                }
            }
        }
        
        // Pour chaque point chaud, si aucun drone à proximité, en rediriger un
        for (int[] hotspot : hotspots) {
            boolean droneNearby = drones.stream()
                .filter(d -> d.getState() == DroneState.ACTIVE)
                .anyMatch(d -> Math.sqrt(Math.pow(d.getX() - hotspot[0], 2) + 
                Math.pow(d.getY() - hotspot[1], 2)) < 5);
            
            if (!droneNearby && !hotspots.isEmpty()) {
                // Trouver un drone actif oisif
                Optional<Drone> idleDrone = drones.stream()
                    .filter(d -> d.getState() == DroneState.ACTIVE)
                    .min(Comparator.comparingDouble(d -> 
                    Math.sqrt(Math.pow(d.getX() - hotspot[0], 2) + 
                    Math.pow(d.getY() - hotspot[1], 2))));
                
                if (idleDrone.isPresent()) {
                    // Créer des points de passage d'urgence vers le point chaud
                    List<double[]> emergency = new ArrayList<>();
                    emergency.add(new double[]{hotspot[0], hotspot[1]});
                    emergency.add(new double[]{0, 0}); // Retour à la base
                    idleDrone.get().setWaypoints(emergency);
                }
            }
        }
    }
    
    public void reset() {
        visitedCells.clear();
        droneWaypoints.clear();
    }
}
