extends Node3D
class_name FlowField

@export var max_climb_height: float = 0.3

var grid_manager: GridManager

var integration_grid: PackedInt32Array = []
var flow_grid: Array[Vector3] = [] # Stores Vector3 directions
var bounds: Rect2i

func _init(_grid_manager: GridManager):
	grid_manager = _grid_manager

func generate_integration_field(target_grid_pos: Vector2i, unit_positions: Array):
	bounds = get_flow_field_bounds(target_grid_pos, unit_positions)
	var size_x = abs(bounds.end.x - bounds.position.x)
	var size_y = abs(bounds.end.y - bounds.position.y)
	
	integration_grid.resize(size_x * size_y)
	flow_grid.resize(size_x * size_y)
	integration_grid.fill(65535) # "Infinity"
	
	var target_index: int = get_local_index(target_grid_pos)
	integration_grid[target_index] = 0
	
	var queue: Array[Vector2i] = [target_grid_pos]
	var head: int = 0
	
	while head < queue.size():
		var current_pos: Vector2i = queue[head]
		head += 1
		
		var current_index: int = get_local_index(current_pos)
		var current_cell: GridManager.CellData = grid_manager.get_cell_value(current_pos)
		var current_integration_value: int = integration_grid[current_index]
		
		for neighbor in _get_8_neighbors_global(current_pos):
			var n_index: int = get_local_index(neighbor)
			var n_cell: GridManager.CellData = grid_manager.get_cell_value(neighbor)
			
			var height_delta = abs(n_cell.height - current_cell.height)
			
			if height_delta > max_climb_height:
				continue
			
			if n_cell.terrain_cost >= 255:
				continue
			
			var step_cost: float = 2.0 if (neighbor.x != current_pos.x and neighbor.y != current_pos.y) else 1.0
			var new_cost: float = current_integration_value + (n_cell.terrain_cost * step_cost)
			var new_cost_int: int = floori(new_cost)
			
			if new_cost_int < integration_grid[n_index]:
				integration_grid[n_index] = new_cost_int
				queue.push_back(neighbor)

func _get_8_neighbors_global(pos: Vector2i) -> Array[Vector2i]:
	var neighbors: Array[Vector2i] = []
	for x in range(-1, 2):
		for y in range(-1, 2):
			if x == 0 and y == 0: continue # Skip self
			
			var check_pos: Vector2i = pos + Vector2i(x, y)
			if _is_within_bounds(check_pos):
				neighbors.append(check_pos)
	return neighbors
			
func generate_flow_field():
	# Resize/clear the flow_grid to match the manager's dimensions
#	flow_grid.resize(grid_manager.grid_width * grid_manager.grid_height)
	for x in range(bounds.position.x, bounds.end.x):
		for y in range(bounds.position.y, bounds.end.y):
			var pos: Vector2i = Vector2i(x, y)
			var current_idx: int = get_local_index(pos)
			
			# 1. Skip if it's a wall or unreachable
			var cell_value: GridManager.CellData = grid_manager.get_cell_value(pos)
			if !cell_value or cell_value.terrain_cost >= 255 or integration_grid[current_idx] >= 65535:
				flow_grid[current_idx] = Vector3.ZERO
				continue
			
			# 2. Find the neighbor with the lowest integration value
			var best_neighbor_pos: Vector2i = pos
			var lowest_cost: int = integration_grid[current_idx]
			
			for neighbor in _get_8_neighbors_global(pos):
				var n_idx: int = get_local_index(neighbor)
				var n_cell: GridManager.CellData = grid_manager.get_cell_value(neighbor)
				var height_delta = abs(n_cell.height - cell_value.height)
				
				if height_delta > max_climb_height:
					continue

				var n_cost: int = integration_grid[n_idx]
				
				if n_cost < lowest_cost:
					lowest_cost = n_cost
					best_neighbor_pos = neighbor
			
			# 3. Calculate the direction vector toward that neighbor
			if best_neighbor_pos != pos:
				# Vector from current tile to the best neighbor
				var dir_2d: Vector2 = Vector2(best_neighbor_pos - pos).normalized()
				# Convert to 3D (X and Z coordinates for the floor plane)
				flow_grid[current_idx] = Vector3(dir_2d.x, 0, dir_2d.y)
			else:
				# We are either at the target or trapped
				flow_grid[current_idx] = Vector3.ZERO
				
func get_flow_at_world_pos(world_pos: Vector3) -> Vector3:
	var grid_pos: Vector2i = grid_manager.world_to_grid(world_pos)
	var index: int = get_local_index(grid_pos)
	if index >= 0 and index < flow_grid.size():
		return flow_grid[index]
	return Vector3.ZERO

func get_flow_field_bounds(target_pos: Vector2i, unit_positions: Array, padding: int = 2) -> Rect2i:
	# 1. Initialize min and max with the target position
	var min_x: int = target_pos.x
	var max_x: int = target_pos.x
	var min_y: int = target_pos.y
	var max_y: int = target_pos.y
	
	# 2. Expand the bounds to include every unit
	for unit_pos in unit_positions:
		min_x = min(min_x, unit_pos.x)
		max_x = max(max_x, unit_pos.x)
		min_y = min(min_y, unit_pos.y)
		max_y = max(max_y, unit_pos.y)
	
	# 3. Add padding so units near the edge have "room" to pathfind around obstacles
	min_x -= padding
	min_y -= padding
	max_x += padding
	max_y += padding
	
	# 4. Clamp to the actual GridManager boundaries to avoid out-of-bounds errors
	if min_x < grid_manager.map_size_min.x:
		min_x = grid_manager.map_size_min.x
	if min_y < grid_manager.map_size_min.y:
		min_y = grid_manager.map_size_min.y
	if max_x > grid_manager.map_size_max.x:
		max_x = grid_manager.map_size_max.x
	if max_y > grid_manager.map_size_max.y:
		max_y = grid_manager.map_size_max.y
	
	# Return as a Rect2i (Position, Size)
	var size_x: int = max_x - min_x
	var size_y: int = max_y - min_y
	return Rect2i(min_x, min_y, size_x, size_y)

func _is_within_bounds(grid_pos: Vector2i) -> bool:
	return grid_pos.x >= bounds.position.x and grid_pos.x < bounds.end.x \
		and grid_pos.y >= bounds.position.y and grid_pos.y < bounds.end.y

func get_local_index(global_pos: Vector2i) -> int:
	var local_x: int = global_pos.x - bounds.position.x
	var local_y: int = global_pos.y - bounds.position.y
	return (local_x + (local_y * bounds.size.x)) -1
	
