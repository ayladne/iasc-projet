# Guide d'Utilisation et Scénarios de Test

## Démarrage Rapide

### 1. Compilation
```bash
mvn clean compile
```

### 2. Lancement
```bash
mvn javafx:run
```

### 3. Interface
- **Canvas central** : Visualisation en temps réel de la grille et des drones
- **Panneau droit** : Contrôles et métriques
- **Zone basse** : Journal des événements

## Codes Couleur

### État des Drones
- 🟢 **Vert** : En exploration (état ACTIVE)
- 🔵 **Bleu** : En mesure d'anomalie (état MEASURING, 10 s)
- 🟠 **Orange** : Retour à la base (état RETURNING)
- 🔴 **Rouge** : En recharge à la base (état CHARGING, 10 min)

### Heatmap d'Anomalies
- 🟡 Jaune : Anomalie faible (0 - 0.5 d'intensité)
- 🟠 Orange : Anomalie modérée (0.5 - 0.75)
- 🔴 Rouge : Anomalie forte (> 0.75)
- 🟦 Base : Zone de départ et recharge (0,0) en vert foncé

## Scénarios de Test Recommandés

### Scénario 1 : Couverture de base (10 min simulation)
**Objectif** : Vérifier que les 7 drones couvrent la zone complète

**Configuration** :
- Pas d'anomalies (probability = 0.0)
- Tous les drones suivent le pattern lawnmower

**Points d'observation** :
- Les drones parcourent la grille en balayage systématique
- Environ 8-10 drones "pixels" par seconde
- Tous retournent à la base pour recharge
- Couverture devrait atteindre ~95-98%

**Métrique clé** : `coveragePercentage` doit tendre vers 100%

---

### Scénario 2 : Anomalies sporadiques (15 min simulation)
**Objectif** : Tester la détection et la communication d'anomalies

**Configuration** :
- Taux d'apparition : 0.05 par tick (~5%)
- Diffusion : 0.10 (faible)
- Décroissance : 0.95 (lente)

**Points d'observation** :
- Les anomalies apparaissent aléatoirement
- Les drones les détectent lors du scan
- Les anomalies se propagent légèrement
- Décroissance visible au fil du temps

**Comportement attendu** :
- Apparitions aléatoires dans la zone
- Heatmap montre des petits clusters
- Anomalies disparaissent progressivement

---

### Scénario 3 : Pollution intense (15 min)
**Objectif** : Tester le comportement du système en cas de forte pollution

**Configuration** :
- Taux d'apparition : 0.15 par tick (~15%)
- Diffusion : 0.15 (moyenne)
- Décroissance : 0.92 (rapide)

**Points d'observation** :
- Multiple anomalies actives simultanément
- Heatmap couverte de zones colorées
- Drones plus souvent en mode MEASURING
- Stress sur la coordination

**Comportement attendu** :
- Couverture d'anomalies très élevée
- Beaucoup plus de détections
- Pattern de retour à base plus fréquent

---

### Scénario 4 : Propagation rapide (20 min)
**Objectif** : Tester les anomalies qui s'étendent rapidement

**Configuration** :
- Taux d'apparition : 0.08 par tick
- Diffusion : 0.20 (forte)
- Décroissance : 0.90 (lente)

**Points d'observation** :
- Anomalies se propagent à travers la grille
- Onde de propagation visible
- Drones détectent des zones élargies
- Nécessite une couverture continus

**Comportement attendu** :
- Anomalies "fusionnent"
- Heatmap montre des zones grandes et continues
- Drones restent plus longtemps en mesure

---

### Scénario 5 : Test d'autonomie énergétique (30 min)
**Objectif** : Vérifier la gestion de l'énergie et le cycle recharge

**Configuration** :
- Configuration normale
- Observer uniquement l'état des drones

**Points d'observation** :
- Chaque drone explore ~18 min avant recharge
- Recharge dure 10 min
- Cycle global : ~28 min par drone
- Staggered returns pour maintenir couverture

**Métriques clés** :
- `activeDrones` : varie entre 4-7
- Jamais 0 drones actifs (idéalement)
- `rechargingDrones` : varie entre 0-3

---

## Métriques à Suivre

### Couverture (Coverage %)
- **Définition** : % de cellules avec anomalies détectées
- **Cible** : 95%+ pour zone sans anomalies
- **Interprétation** : Plus haut = meilleure exploration

### Anomalies Détectées (Anomalies)
- **Définition** : Nombre de sources d'anomalies actives
- **Cible** : Proportionnel au taux d'apparition
- **Interprétation** : Reflète l'état de l'environnement

### Drones Actifs (Active)
- **Définition** : Nombre de drones en exploration
- **Cible** : 4-7 (jamais 0)
- **Interprétation** : Drones en cycle normal

### Drones en Recharge (Charging)
- **Définition** : Nombre de drones à la base en recharge
- **Cible** : 0-2 (variable)
- **Interprétation** : État du cycle énergétique

---

## Conseils pour la Présentation (15 min)

### Structure Recommandée

**1. Introduction (2 min)**
- Contexte : surveillance de zones sensibles
- Problème : besoin d'autonomie + coordination
- Solution : essaim de 7 drones coopératifs

**2. Architecture Modèle (3 min)**
- Environnement : grille 50×50, anomalies dynamiques
- Drone : état (actif/mesure/retour/recharge), autonomie limitée
- Communication : uplink direct, downlink à la base

**3. Algorithmes & Stratégies (3 min)**
- Couverture : pattern lawnmower
- Détection : seuil intensité 0.3
- Dynamique anomalie : diffusion + décroissance
- Énergie : 30 min actif, 10 min recharge

**4. Résultats & Démonstration (4 min)**
- Lancer la simulation live sur projecteur
- Montrer un scénario complet (5-10 min accelérées)
- Afficher les graphes de métriques

**5. Conclusion (1-2 min)**
- Points forts : modularité, extensibilité
- Limites : pas d'obstacles, pattern statique
- Améliorations futures : IA adaptative, obstacles, déploiement réel

---

## Paramètres Ajustables (Optionnel)

Pour personnaliser les scénarios, modifier `SimulationParams.java` :

```java
// Drone parameters
public static final int DRONE_AUTONOMY_MS = 30 * 60 * 1000; // Changer autonomie
public static final int NUM_DRONES = 7; // Augmenter/diminuer nombre

// Anomaly parameters
public static final double ANOMALY_SPAWN_PROBABILITY = 0.05; // Taux apparition
public static final double ANOMALY_DIFFUSION_FACTOR = 0.1; // Diffusion
public static final double ANOMALY_DECAY_RATE = 0.95; // Décroissance
```

Puis recompiler et relancer.

---

## Fichiers de Sortie

Les méthodes d'export CSV sont disponibles dans `ExportUtils.java` :
- `exportMetricsToCSV()` : Exporte les métriques par timestep
- `exportMeasurementsToCSV()` : Exporte les mesures par drone

À intégrer dans l'UI pour générer graphes et rapports.

---

## Questions Courantes

**Q: Pourquoi les drones ne retournent pas tous en même temps ?**  
A: Le pattern lawnmower est partitionné par drone. Chacun couvre sa région à son rythme, donc les retours sont échelonnés.

**Q: Les anomalies disparaissent trop vite ?**  
A: Réduire `ANOMALY_DECAY_RATE` (ex. 0.90 au lieu de 0.95).

**Q: Les drones se chevaucher ?**  
A: Pas d'anticollision programmée. Une simple heuristique (réserver cellules) est possible.

**Q: Comment ajouter des obstacles ?**  
A: Modifier `Environment` pour marquér des cellules "interdites", puis adapter pathfinding dans `Drone.moveToward()`.

---

## Checklist de Démonstration

- [ ] Projet compile sans erreur (`mvn clean compile`)
- [ ] UI lance sans crash (`mvn javafx:run`)
- [ ] Boutons Start/Stop/Reset fonctionnent
- [ ] Grille affiche correctement
- [ ] Drones se déplacent visiblement
- [ ] Heatmap se met à jour
- [ ] Métriques changent au fil du temps
- [ ] Log affiche les événements clés
- [ ] Simulation peut tourner 15+ min sans problème

---

Bonne chance pour votre présentation ! 🚁
