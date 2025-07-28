import json
import math
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from collections import defaultdict

@dataclass
class LandmarkObservation:
    """Represents a landmark observation with distance and direction."""
    id: str
    distance: int
    direction: str
    relative_pos: Tuple[int, int] = field(init=False)
    
    def __post_init__(self):
        self.relative_pos = self._calculate_relative_position()
    
    def _calculate_relative_position(self) -> Tuple[int, int]:
        """Convert direction and distance to relative (dx, dy) coordinates."""
        direction_vectors = {
            'Current': (0, 0),
            'North': (0, -1),
            'Northeast': (1, -1),
            'East': (1, 0),
            'Southeast': (1, 1),
            'South': (0, 1),
            'Southwest': (-1, 1),
            'West': (-1, 0),
            'Northwest': (-1, -1)
        }
        
        if self.direction not in direction_vectors:
            return (0, 0)
        
        dx, dy = direction_vectors[self.direction]
        return (dx * self.distance, dy * self.distance)

@dataclass
class TerrainInfo:
    """Information about a grid cell."""
    terrain_type: str
    slope: Optional[str] = None
    is_passable: bool = True
    visibility_distance: Optional[int] = None

class SimpleSLAM:
    """
    Simple SLAM algorithm for grid-based exploration with unique landmark IDs.
    
    Assumes:
    - 8-directional movement on a grid
    - 360-degree visibility with compass directions
    - Unique, persistent landmark IDs
    - Known movement directions
    """
    
    def __init__(self, grid_size: int = 50):
        # Position tracking
        self.position: Optional[Tuple[int, int]] = None
        self.is_initialized = False
        
        # Maps and databases
        self.global_landmarks: Dict[str, Tuple[int, int]] = {}
        self.visited_locations: Dict[Tuple[int, int], Any] = {}
        self.landmark_patterns: Dict[Tuple[int, int], Dict[str, Tuple[int, int]]] = {}
        self.occupancy_grid: Dict[Tuple[int, int], TerrainInfo] = {}
        
        # Configuration
        self.grid_size = grid_size
        
        # Statistics
        self.step_count = 0
        self.loop_closures = 0
        self.position_corrections = 0
    
    def process_step(self, move_data: Optional[Dict], look_data: Dict) -> Dict[str, Any]:
        """
        Process a single step of movement and observation.
        
        Args:
            move_data: {"direction": "north"} or None for first step
            look_data: Full observation data
            
        Returns:
            Dictionary with processing results and current state
        """
        self.step_count += 1
        
        # Step 1: Update position from movement
        if move_data and self.is_initialized:
            self._dead_reckon(move_data['direction'])
        
        # Step 2: Extract landmarks from observations
        observed_landmarks = self._extract_landmarks(look_data)
        
        # Step 3: Initialize position if this is the first step
        if not self.is_initialized and observed_landmarks:
            self._initialize_position(observed_landmarks)
        
        # Step 4: Attempt loop closure
        corrected_position = None
        if self.is_initialized:
            corrected_position = self._attempt_loop_closure(observed_landmarks)
            if corrected_position:
                self.position = corrected_position
                self.loop_closures += 1
                self.position_corrections += 1
        
        # Step 5: Update all maps with current position
        if self.is_initialized:
            self._update_maps(look_data, observed_landmarks)
        
        # Return status information
        return {
            'step': self.step_count,
            'position': self.position,
            'initialized': self.is_initialized,
            'landmarks_observed': len(observed_landmarks),
            'total_landmarks': len(self.global_landmarks),
            'visited_cells': len(self.visited_locations),
            'loop_closure': corrected_position is not None,
            'loop_closures_total': self.loop_closures
        }
    
    def _dead_reckon(self, direction: str) -> None:
        """Update position based on movement direction."""
        if not self.position:
            return
        
        direction_vectors = {
            'north': (0, -1),
            'northeast': (1, -1),
            'east': (1, 0),
            'southeast': (1, 1),
            'south': (0, 1),
            'southwest': (-1, 1),
            'west': (-1, 0),
            'northwest': (-1, -1)
        }
        
        direction_lower = direction.lower()
        if direction_lower in direction_vectors:
            dx, dy = direction_vectors[direction_lower]
            x, y = self.position
            self.position = (x + dx, y + dy)
    
    def _extract_landmarks(self, look_data: Dict) -> List[LandmarkObservation]:
        """Extract landmark observations from look data."""
        landmarks = []
        
        # Process each direction
        directions = look_data.get('directions', {})
        for direction, dir_data in directions.items():
            resources = dir_data.get('resources', [])
            for resource in resources:
                landmark = LandmarkObservation(
                    id=resource['id'],
                    distance=resource['distance'],
                    direction=direction
                )
                landmarks.append(landmark)
        
        # Add adjacent resources (distance 0, current location)
        adjacent = look_data.get('adjacent_resources', [])
        for resource_id in adjacent:
            landmark = LandmarkObservation(
                id=resource_id,
                distance=0,
                direction='Current'
            )
            landmarks.append(landmark)
        
        return landmarks
    
    def _initialize_position(self, observed_landmarks: List[LandmarkObservation]) -> None:
        """Initialize global position using first landmark observations."""
        if not observed_landmarks:
            return
        
        # Arbitrarily set current position to center of grid
        self.position = (self.grid_size // 2, self.grid_size // 2)
        self.is_initialized = True
        
        # Add landmarks to global database based on this position
        for landmark in observed_landmarks:
            global_pos = self._relative_to_global(landmark.relative_pos, self.position)
            self.global_landmarks[landmark.id] = global_pos
    
    def _attempt_loop_closure(self, observed_landmarks: List[LandmarkObservation]) -> Optional[Tuple[int, int]]:
        """
        Attempt to find current position by matching landmark patterns.
        
        Returns corrected position if confident match found, None otherwise.
        """
        if len(observed_landmarks) < 2:  # Need multiple landmarks for confident match
            return None
        
        # Create current landmark pattern
        current_pattern = {
            lm.id: lm.relative_pos for lm in observed_landmarks
        }
        
        # Check against all previously visited locations
        for prev_location, prev_pattern in self.landmark_patterns.items():
            if self._patterns_match(current_pattern, prev_pattern):
                return prev_location
        
        # Also check if we can match against known global landmarks
        estimated_position = self._estimate_position_from_landmarks(observed_landmarks)
        if estimated_position and self._validate_position_estimate(estimated_position, observed_landmarks):
            return estimated_position
        
        return None
    
    def _patterns_match(self, pattern1: Dict[str, Tuple[int, int]], 
                       pattern2: Dict[str, Tuple[int, int]], 
                       tolerance: int = 0) -> bool:
        """Check if two landmark patterns match within tolerance."""
        # Must have significant overlap
        common_landmarks = set(pattern1.keys()) & set(pattern2.keys())
        if len(common_landmarks) < 2:
            return False
        
        # Check if relative positions match for common landmarks
        for landmark_id in common_landmarks:
            pos1 = pattern1[landmark_id]
            pos2 = pattern2[landmark_id]
            if abs(pos1[0] - pos2[0]) > tolerance or abs(pos1[1] - pos2[1]) > tolerance:
                return False
        
        return True
    
    def _estimate_position_from_landmarks(self, observed_landmarks: List[LandmarkObservation]) -> Optional[Tuple[int, int]]:
        """Estimate current position using known global landmark positions."""
        estimates = []
        
        for landmark in observed_landmarks:
            if landmark.id in self.global_landmarks:
                global_lm_pos = self.global_landmarks[landmark.id]
                # Current position = landmark_global_pos - landmark_relative_pos
                estimated_pos = (
                    global_lm_pos[0] - landmark.relative_pos[0],
                    global_lm_pos[1] - landmark.relative_pos[1]
                )
                estimates.append(estimated_pos)
        
        if not estimates:
            return None
        
        # If multiple estimates, check for consistency
        if len(estimates) == 1:
            return estimates[0]
        
        # Check if all estimates agree (or are very close)
        first_estimate = estimates[0]
        for estimate in estimates[1:]:
            if abs(estimate[0] - first_estimate[0]) > 1 or abs(estimate[1] - first_estimate[1]) > 1:
                return None  # Inconsistent estimates
        
        return first_estimate
    
    def _validate_position_estimate(self, position: Tuple[int, int], 
                                   observed_landmarks: List[LandmarkObservation]) -> bool:
        """Validate position estimate against all observed landmarks."""
        for landmark in observed_landmarks:
            if landmark.id in self.global_landmarks:
                expected_global_pos = self._relative_to_global(landmark.relative_pos, position)
                actual_global_pos = self.global_landmarks[landmark.id]
                
                if expected_global_pos != actual_global_pos:
                    return False
        
        return True
    
    def _update_maps(self, look_data: Dict, observed_landmarks: List[LandmarkObservation]) -> None:
        """Update occupancy grid, landmark database, and pattern database."""
        if not self.position:
            return
        
        # Update visited locations
        self.visited_locations[self.position] = {
            'step': self.step_count,
            'landmarks': [lm.id for lm in observed_landmarks]
        }
        
        # Update landmark pattern database
        landmark_pattern = {lm.id: lm.relative_pos for lm in observed_landmarks}
        self.landmark_patterns[self.position] = landmark_pattern
        
        # Update global landmark database
        for landmark in observed_landmarks:
            global_pos = self._relative_to_global(landmark.relative_pos, self.position)
            if landmark.id not in self.global_landmarks:
                self.global_landmarks[landmark.id] = global_pos
        
        # Update occupancy grid from visibility data
        self._update_occupancy_grid(look_data)
    
    def _update_occupancy_grid(self, look_data: Dict) -> None:
        """Update occupancy grid based on visibility information."""
        if not self.position:
            return
        
        directions = look_data.get('directions', {})
        
        for direction, dir_data in directions.items():
            terrain = dir_data.get('terrain')
            visibility = dir_data.get('visibility', 0)
            slope = dir_data.get('slope')
            
            if direction == 'Current':
                # Update current cell
                terrain_info = TerrainInfo(
                    terrain_type=terrain,
                    slope=slope,
                    visibility_distance=visibility
                )
                self.occupancy_grid[self.position] = terrain_info
            else:
                # Update cells along visibility ray
                direction_vector = self._get_direction_vector(direction)
                if direction_vector:
                    self._update_visibility_ray(direction_vector, visibility, terrain, slope)
    
    def _update_visibility_ray(self, direction_vector: Tuple[int, int], 
                              visibility: int, terrain: str, slope: Optional[str]) -> None:
        """Update occupancy grid along a visibility ray."""
        if not self.position:
            return
        
        dx, dy = direction_vector
        x, y = self.position
        
        # Mark cells along the ray as passable (up to visibility distance)
        for distance in range(1, min(visibility + 1, self.grid_size)):
            cell_x = x + dx * distance
            cell_y = y + dy * distance
            
            # Check bounds
            if 0 <= cell_x < self.grid_size and 0 <= cell_y < self.grid_size:
                cell_pos = (cell_x, cell_y)
                if cell_pos not in self.occupancy_grid:
                    # Mark as passable with the observed terrain type
                    self.occupancy_grid[cell_pos] = TerrainInfo(
                        terrain_type=terrain,
                        slope=slope,
                        is_passable=True
                    )
        
        # If visibility is limited, mark the blocking cell
        if visibility < self.grid_size:
            block_x = x + dx * (visibility + 1)
            block_y = y + dy * (visibility + 1)
            if 0 <= block_x < self.grid_size and 0 <= block_y < self.grid_size:
                block_pos = (block_x, block_y)
                if block_pos not in self.occupancy_grid:
                    # Mark as potentially impassable (or different terrain)
                    self.occupancy_grid[block_pos] = TerrainInfo(
                        terrain_type="Unknown",
                        is_passable=False
                    )
    
    def _get_direction_vector(self, direction: str) -> Optional[Tuple[int, int]]:
        """Get unit direction vector for a compass direction."""
        direction_vectors = {
            'North': (0, -1),
            'Northeast': (1, -1),
            'East': (1, 0),
            'Southeast': (1, 1),
            'South': (0, 1),
            'Southwest': (-1, 1),
            'West': (-1, 0),
            'Northwest': (-1, -1)
        }
        return direction_vectors.get(direction)
    
    def _relative_to_global(self, relative_pos: Tuple[int, int], 
                           current_pos: Tuple[int, int]) -> Tuple[int, int]:
        """Convert relative position to global coordinates."""
        rel_x, rel_y = relative_pos
        cur_x, cur_y = current_pos
        return (cur_x + rel_x, cur_y + rel_y)
    
    def get_map_summary(self) -> Dict[str, Any]:
        """Get summary of current map state."""
        return {
            'position': self.position,
            'initialized': self.is_initialized,
            'landmarks_known': len(self.global_landmarks),
            'cells_visited': len(self.visited_locations),
            'cells_mapped': len(self.occupancy_grid),
            'loop_closures': self.loop_closures,
            'steps': self.step_count,
            'landmarks': dict(self.global_landmarks),
            'coverage': len(self.visited_locations) / (self.grid_size * self.grid_size) * 100
        }
    
    def get_landmark_at_position(self, position: Tuple[int, int]) -> Optional[str]:
        """Get landmark ID at a specific global position."""
        for landmark_id, landmark_pos in self.global_landmarks.items():
            if landmark_pos == position:
                return landmark_id
        return None


# Example usage and testing
def example_usage():
    """Example of how to use the SimpleSLAM class."""
    slam = SimpleSLAM(grid_size=20)  # Start with smaller grid for testing
    
    # Example input data (first step - no movement)
    look_data_1 = {
        "directions": {
            "Current": {"terrain": "Forest", "visibility": 0},
            "North": {"terrain": "Forest", "visibility": 19, "slope": "Downhill",
                     "resources": [{"id": "Mushrooms2", "distance": 3}]},
            "Northwest": {"terrain": "Forest", "visibility": 27, "slope": "Downhill",
                         "resources": [{"id": "Berries7", "distance": 6}, {"id": "Apple_Tree9", "distance": 8}]}
        },
        "adjacent_resources": []
    }
    
    # Process first step (no movement)
    result1 = slam.process_step(None, look_data_1)
    print("Step 1:", result1)
    
    # Example second step (move north)
    move_data_2 = {"direction": "north"}
    look_data_2 = {
        "directions": {
            "Current": {"terrain": "Forest", "visibility": 0},
            "North": {"terrain": "Forest", "visibility": 16, "slope": "Downhill",
                     "resources": [{"id": "Mushrooms2", "distance": 2}]},
            "South": {"terrain": "Forest", "visibility": 20, "slope": "Uphill"}
        },
        "adjacent_resources": []
    }
    
    result2 = slam.process_step(move_data_2, look_data_2)
    print("Step 2:", result2)
    
    # Move closer to Mushrooms2
    move_data_3 = {"direction": "north"}
    look_data_3 = {
        "directions": {
            "Current": {"terrain": "Forest", "visibility": 0},
            "North": {"terrain": "Forest", "visibility": 15, "slope": "Downhill",
                     "resources": [{"id": "Mushrooms2", "distance": 1}]},
            "South": {"terrain": "Forest", "visibility": 21, "slope": "Uphill"}
        },
        "adjacent_resources": []
    }
    
    result3 = slam.process_step(move_data_3, look_data_3)
    print("Step 3:", result3)
    
    # Move to Mushrooms2 location
    move_data_4 = {"direction": "north"}
    look_data_4 = {
        "directions": {
            "Current": {"terrain": "Forest", "visibility": 0},
            "South": {"terrain": "Forest", "visibility": 22, "slope": "Uphill"}
        },
        "adjacent_resources": ["Mushrooms2"]
    }
    
    result4 = slam.process_step(move_data_4, look_data_4)
    print("Step 4:", result4)
    
    # Now move back south to test loop closure
    move_data_5 = {"direction": "south"}
    look_data_5 = {
        "directions": {
            "Current": {"terrain": "Forest", "visibility": 0},
            "North": {"terrain": "Forest", "visibility": 15, "slope": "Downhill",
                     "resources": [{"id": "Mushrooms2", "distance": 1}]},
            "South": {"terrain": "Forest", "visibility": 21, "slope": "Uphill"}
        },
        "adjacent_resources": []
    }
    
    result5 = slam.process_step(move_data_5, look_data_5)
    print("Step 5 (should detect loop closure):", result5)
    
    # Get map summary
    summary = slam.get_map_summary()
    print("\nFinal Map Summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    example_usage()