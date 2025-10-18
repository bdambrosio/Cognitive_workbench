"""
map_agent.py - Map navigation proxy for characters

Provides the MapAgent class which serves as a spatial representation
of scenario characters, handling their position, movement, and visibility
within any SpaceMap subclass (WorldMap, InfospaceMap, etc.).

MapAgent is NOT a cognitive agent - it's a map subsystem proxy that:
- Tracks character position in spatial coordinates
- Handles movement through the grid
- Queries visibility and line-of-sight
- Interfaces with map resources

Cognitive/character logic lives in other subsystems (executive_node, 
memory_node, perception_node, etc.).
"""

from enum import Enum, auto
import random


class Direction(Enum):
    """Cardinal and ordinal directions plus current position"""
    Current = auto()
    North = auto()
    Northeast = auto()
    East = auto()
    Southeast = auto()
    South = auto()
    Southwest = auto()
    West = auto()
    Northwest = auto()

    @staticmethod
    def from_string(text):
        """
        Convert text to Direction enum, with enhanced robustness
        
        Args:
            text: String or Direction enum to convert
            
        Returns:
            Direction enum or None if no valid direction found
        """
        # Return as-is if already a Direction
        if isinstance(text, Direction):
            return text
            
        # Handle None/empty input
        if not text:
            return None
            
        # Convert to lowercase string for matching
        if not isinstance(text, str):
            return None
            
        text = text.lower().strip()
        
        # Handle common variations
        direction_map = {
            'n': Direction.North,
            'ne': Direction.Northeast, 
            'e': Direction.East,
            'se': Direction.Southeast,
            's': Direction.South, 
            'sw': Direction.Southwest,
            'w': Direction.West,
            'nw': Direction.Northwest,
            'north': Direction.North,
            'northeast': Direction.Northeast,
            'east': Direction.East, 
            'southeast': Direction.Southeast,
            'south': Direction.South,
            'southwest': Direction.Southwest,
            'west': Direction.West,
            'northwest': Direction.Northwest
        }
        
        # Try exact match first
        if text in direction_map:
            return direction_map[text]
            
        # Try matching parts of input against direction names
        for name, direction in direction_map.items():
            if name == text:
                return direction
                
        return None


def get_direction_offset(direction):
    """
    Get x,y offset for a direction
    
    Args:
        direction: Direction enum or string
        
    Returns:
        Tuple of (dx, dy) offsets or (0,0) if invalid
    """
    # Convert string to enum if needed
    if isinstance(direction, str):
        direction = Direction.from_string(direction)
        
    # Return no movement if invalid direction
    if direction is None:
        return (0, 0)
        
    offsets = {
        Direction.North: (0, -1),
        Direction.Northeast: (1, -1),
        Direction.East: (1, 0),
        Direction.Southeast: (1, 1),
        Direction.South: (0, 1),
        Direction.Southwest: (-1, 1),
        Direction.West: (-1, 0),
        Direction.Northwest: (-1, -1),
        Direction.Current: (0, 0)
    }
    
    return offsets[direction]


# --- module-level pre-computed thresholds -------------------------------
_TAN_22_5 = 0.4142135623730950   # tan(22.5°)  = √2-1
_TAN_67_5 = 2.4142135623730950   # tan(67.5°)  = √2+1
# -----------------------------------------------------------------------


def get_direction_name(dx: float, dy: float):
    """
    Return Direction enum for the displacement (dx, dy) without calling atan/atan2.

    The plane is sliced into three bands per quadrant:
        • near vertical    |dx/dy| ≤ tan 22.5°  →  N / S
        • diagonal         tan 22.5° < |dx/dy| < tan 67.5° → NE / SE / SW / NW
        • near horizontal  |dx/dy| ≥ tan 67.5°  →  E / W
    
    Args:
        dx: X displacement
        dy: Y displacement
        
    Returns:
        Direction enum representing the direction
    """
    if dx == dy == 0:
        return Direction.Current   # no direction

    ax, ay = abs(dx), abs(dy)          # speed: no divisions, only compares

    if ax <= ay * _TAN_22_5:           # "mostly north/south"
        return Direction.North if dy > 0 else Direction.South

    if ax >= ay * _TAN_67_5:           # "mostly east/west"
        return Direction.East if dx > 0 else Direction.West

    # diagonal band
    if dx > 0 and dy > 0:   
        return Direction.Northeast
    if dx > 0 and dy < 0:   
        return Direction.Southeast
    if dx < 0 and dy < 0:   
        return Direction.Southwest
    # else: dx < 0 and dy > 0
    return Direction.Northwest


class MapAgent:
    """
    Map navigation proxy for a scenario character.
    
    Represents a character's position and provides methods for spatial
    interaction with any SpaceMap subclass. Works polymorphically with
    WorldMap, InfospaceMap, or any future spatial map types.
    
    This is a thin spatial layer - reasoning, memory, and personality
    are handled by other subsystems (executive_node, memory_node, etc.).
    
    Attributes:
        x, y: Current position coordinates
        world: Reference to the SpaceMap instance
        name: Character name
        _last_path_dir: Last path traversal direction (for path following)
    """
    
    def __init__(self, x, y, world, name):
        """
        Initialize a map agent at a position.
        
        Args:
            x, y: Starting coordinates
            world: SpaceMap instance (WorldMap, InfospaceMap, etc.)
            name: Character name
        """
        self.x = x
        self.y = y
        self.world = world
        self.name = name
        self.world.register_agent(self)
        # Track last path traversal direction as (dx, dy)
        self._last_path_dir = None

    def look(self):
        """
        Get detailed visibility description from current position.
        
        Returns:
            XML string describing visible patches, resources, and agents
        """
        return self.world.get_detailed_visibility_description(self.x, self.y, self, 5)

    def local_map(self):
        """
        Get local map view (alias for look()).
        
        Returns:
            XML string describing visible area
        """
        return self.world.get_detailed_visibility_description(self.x, self.y, self, 5)

    def move(self, direction):
        """
        Move one step in a direction.
        
        Args:
            direction: Direction enum or string ('north', 'ne', etc.)
            
        Returns:
            True if move succeeded, False if blocked or out of bounds
        """
        direction = Direction.from_string(direction)
        if not direction:
            return False
        dx, dy = self.get_direction_offset(direction)
        new_x, new_y = self.x + dx, self.y + dy

        # Option 1: Wrapping edges
        # new_x = new_x % self.world.width
        # new_y = new_y % self.world.height

        # Option 2: Hard boundaries
        if 0 <= new_x < self.world.width and 0 <= new_y < self.world.height:
            self.x, self.y = new_x, new_y
            return True
        else:
            return False

    def __del__(self):
        """Cleanup: unregister from world when agent is destroyed"""
        self.world.unregister_agent(self)

    @staticmethod
    def get_direction_offset(direction_text):
        """
        Get x,y offset for a direction, handling natural language descriptions.
        
        Args:
            direction_text: String that may contain a direction within it
            
        Returns:
            Tuple of (dx, dy) offsets or (0,0) if no valid direction found
        """
        # First try direct conversion
        direction = Direction.from_string(direction_text)
        if direction:
            offsets = {
                Direction.North: (0, 1),
                Direction.Northeast: (1, 1),
                Direction.East: (1, 0),
                Direction.Southeast: (1, -1),
                Direction.South: (0, -1),
                Direction.Southwest: (-1, -1),
                Direction.West: (-1, 0),
                Direction.Northwest: (-1, -1)
            }
            return offsets[direction]

        # If that fails, look for direction words in the text
        direction_words = {
            'north': (0, 1),
            'northeast': (1, 1), 
            'east': (1, 0),
            'southeast': (1, -1),
            'south': (0, -1),
            'southwest': (-1, -1),
            'west': (-1, 0),
            'northwest': (-1, 1),
            # Add common abbreviations
            'n': (0, 1),
            'ne': (1, 1),
            'e': (1, 0),
            'se': (1, -1),
            's': (0, -1),
            'sw': (-1, -1),
            'w': (-1, 0),
            'nw': (-1, 1)
        }

        # Convert to lowercase and split into words
        words = direction_text.lower().split()
        
        # Look for any direction word in the text
        for word in words:
            if word in direction_words:
                return direction_words[word]
                
        # If no direction found, return no movement
        return (0, 0)

    def get_detailed_visibility_description(self, height=5):
        """
        Get detailed visibility description with specified observer height.
        
        Args:
            height: Observer height (affects visibility in physical spaces)
            
        Returns:
            XML string describing visibility
        """
        return self.world.get_detailed_visibility_description(self.x, self.y, self, height)

    def move_to_resource(self, resource_id):
        """
        Teleport agent directly to resource location.
        
        Args:
            resource_id: ID of resource to move to
            
        Returns:
            True if successful, False if resource not found
        """
        if resource_id not in self.world.resource_registry:
            print(f"ERROR: Resource {resource_id} not found")
            return False
            
        x, y = self.world.resource_registry[resource_id]['location']
        self.x = x
        self.y = y
        return True

    def direction_toward(self, resource_id):
        """
        Get direction name toward a resource.
        
        Args:
            resource_id: ID of target resource
            
        Returns:
            Direction name string or False if resource not found
        """
        if resource_id in self.world.resource_registry:
            target_x, target_y = self.world.resource_registry[resource_id]['location']
        else:
            print(f"ERROR: Resource {resource_id} not found")
            return False
        target_x, target_y = self.world.resource_registry[resource_id]['location']
        dx = target_x - self.x
        dy = target_y - self.y
        direction = get_direction_name(dx, dy)
        return direction

    def move_toward(self, resource_id):
        """
        Move one step toward a resource.
        
        Args:
            resource_id: ID of target resource
            
        Returns:
            True if move succeeded, False if failed or resource not found
        """
        if resource_id in self.world.resource_registry:
            target_x, target_y = self.world.resource_registry[resource_id]['location']
        else:
            print(f"ERROR: Resource {resource_id} not found")
            return False
        target_x, target_y = self.world.resource_registry[resource_id]['location']
        return self.move_toward_location(target_x, target_y)
                    
    def move_toward_location(self, target_x, target_y):
        """
        Move one step toward a target location.
        
        Args:
            target_x, target_y: Target coordinates
            
        Returns:
            True if move succeeded or already at target
        """
        # Already there
        if (self.x, self.y) == (target_x, target_y):
            return True
            
        # Get direction to target
        dx = target_x - self.x
        dy = target_y - self.y
        direction = get_direction_name(dx, dy)
        
        # Use existing move method
        return self.move(direction)

    def use_path(self, path_id: str) -> bool:
        """
        Traverse a path infrastructure element.
        
        If near but not on path, moves to a patch across the path that also has path.
        If already on the path patch, continues in the last path direction if possible.
        Falls back to stepping onto the path patch if no across candidate exists.
        
        Note: Only meaningful for maps with infrastructure (WorldMap). 
        Returns False for information spaces.
        
        Args:
            path_id: Path identifier (e.g., "Path23")
            
        Returns:
            True if successfully traversed path, False otherwise
        """
        # Resolve PathXY to coordinates by matching against patches
        target_xy = None
        for x in range(self.world.width):
            for y in range(self.world.height):
                p = self.world.patches[x][y]
                if p.has_path and f"Path{p.x}{p.y}" == path_id:
                    target_xy = (p.x, p.y)
                    break
            if target_xy:
                break
        if target_xy is None:
            return False

        tx, ty = target_xy
        ax, ay = self.x, self.y

        # If already on the path patch: try continuing in stored direction
        if (ax, ay) == (tx, ty):
            dx, dy = self._last_path_dir if self._last_path_dir else (0, 0)
            nx, ny = ax + dx, ay + dy
            if 0 <= nx < self.world.width and 0 <= ny < self.world.height and self.world.patches[nx][ny].has_path:
                self.x, self.y = nx, ny
                return True
            # Fallback: try any neighbor with path, prefer forward-ish
            candidates = []
            for ox in (-1, 0, 1):
                for oy in (-1, 0, 1):
                    if ox == 0 and oy == 0:
                        continue
                    nx, ny = ax + ox, ay + oy
                    if 0 <= nx < self.world.width and 0 <= ny < self.world.height and self.world.patches[nx][ny].has_path:
                        # Score by alignment with previous direction (dot product)
                        score = ox * (dx or 0) + oy * (dy or 0)
                        candidates.append(((nx, ny), score))
            if candidates:
                # Pick best alignment; if tie, random among best
                max_score = max(s for _, s in candidates)
                best = [pos for pos, s in candidates if s == max_score]
                nx, ny = random.choice(best)
                self._last_path_dir = (nx - ax, ny - ay)
                self.x, self.y = nx, ny
                return True
            return False

        # Not on path: compute across-path candidates relative to agent->path vector
        vx = tx - ax
        vy = ty - ay
        sx = 1 if vx > 0 else (-1 if vx < 0 else 0)
        sy = 1 if vy > 0 else (-1 if vy < 0 else 0)

        cand_coords = []
        if sx != 0 and sy != 0:
            cand_coords = [(tx + sx, ty + sy)]
        elif sx == 0 and sy != 0:
            cand_coords = [(tx + 1, ty + sy), (tx - 1, ty + sy)]
        elif sy == 0 and sx != 0:
            cand_coords = [(tx + sx, ty + 1), (tx + sx, ty - 1)]

        # Filter to in-bounds path patches
        cand_coords = [c for c in cand_coords if 0 <= c[0] < self.world.width and 0 <= c[1] < self.world.height and self.world.patches[c[0]][c[1]].has_path]

        chosen = None
        if cand_coords:
            # Choose nearest to ideal across point (tx+sx, ty+sy) by Euclidean; tie -> random
            ideal = (tx + (sx or 0), ty + (sy or 0))
            def ed2(c):
                return (c[0] - ideal[0]) ** 2 + (c[1] - ideal[1]) ** 2
            bestd = min(ed2(c) for c in cand_coords)
            bests = [c for c in cand_coords if ed2(c) == bestd]
            chosen = random.choice(bests)
        else:
            # No across candidate: step onto the path patch itself
            chosen = (tx, ty)

        nx, ny = chosen
        self._last_path_dir = (nx - tx, ny - ty) if (nx, ny) != (tx, ty) else (sx, sy)
        self.x, self.y = nx, ny
        return True

