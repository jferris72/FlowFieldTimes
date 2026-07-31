using System;
using System.Collections.Generic;
using Godot;
using RTSTest.function_nodes.grid;

namespace RTSTest.function_nodes.flowfield;

public partial class FlowFieldC : RefCounted
{
	private float _maxClimbHeight = 0.3f;
	private GridManagerC _gridManager;

	private int[] _integrationGrid;
	private Vector3[] _flowGrid;
	private Rect2I _bounds;

	public FlowFieldC(GridManagerC manager)
	{
		_gridManager = manager;
	}

	public void GenerateIntegrationField(Vector2I targetGridPos, List<Vector2I> unitPositions)
	{
		_bounds = GetFlowFieldBounds(targetGridPos, unitPositions);
		
		int sizeX = _bounds.Size.X;
		int sizeY = _bounds.Size.Y;
		int totalCells = sizeX * sizeY;

		_integrationGrid = new int[totalCells];
		_flowGrid = new Vector3[totalCells];
		
		// Fill with "Infinity" (using a high number like 65535)
		Array.Fill(_integrationGrid, 65535);

		int targetIndex = GetLocalIndex(targetGridPos);
		if (targetIndex == -1) return;

		_integrationGrid[targetIndex] = 0;

		// C# Queue is significantly faster than GDScript Array for Dijkstra
		Queue<Vector2I> queue = new Queue<Vector2I>();
		queue.Enqueue(targetGridPos);

		while (queue.Count > 0)
		{
			Vector2I currentPos = queue.Dequeue();
			int currentIndex = GetLocalIndex(currentPos);
			
			// In C#, structs are passed by value, so we get a copy
			GridManagerC.CellData currentCell = _gridManager.GetCellValue(currentPos);
			int currentIntegrationValue = _integrationGrid[currentIndex];

			foreach (Vector2I neighbor in Get8NeighborsGlobal(currentPos))
			{
				int nIndex = GetLocalIndex(neighbor);
				GridManagerC.CellData nCell = _gridManager.GetCellValue(neighbor);

				float heightDelta = Math.Abs(nCell.Height - currentCell.Height);

				if (heightDelta > _maxClimbHeight)
					continue;

				if (nCell.TerrainCost >= 255)
					continue;

				// Diagonal cost is roughly 1.4, but using 2 for integer Dijkstra is fine
				float stepCost = (neighbor.X != currentPos.X && neighbor.Y != currentPos.Y) ? 1.4f : 1.0f;
				int newCost = (int)(currentIntegrationValue + (nCell.TerrainCost * stepCost));

				if (newCost < _integrationGrid[nIndex])
				{
					_integrationGrid[nIndex] = newCost;
					queue.Enqueue(neighbor);
				}
			}
		}
	}

	public void GenerateFlowField()
	{
		for (int x = _bounds.Position.X; x < _bounds.End.X; x++)
		{
			for (int y = _bounds.Position.Y; y < _bounds.End.Y; y++)
			{
				Vector2I pos = new Vector2I(x, y);
				int currentIdx = GetLocalIndex(pos);

				GridManagerC.CellData cellValue = _gridManager.GetCellValue(pos);
				
				// Skip if wall or unreachable
				if (cellValue.TerrainCost >= 255 || _integrationGrid[currentIdx] >= 65535)
				{
					_flowGrid[currentIdx] = Vector3.Zero;
					continue;
				}

				Vector2I bestNeighborPos = pos;
				int lowestCost = _integrationGrid[currentIdx];

				foreach (Vector2I neighbor in Get8NeighborsGlobal(pos))
				{
					int nIdx = GetLocalIndex(neighbor);
					GridManagerC.CellData nCell = _gridManager.GetCellValue(neighbor);
					
					if (Math.Abs(nCell.Height - cellValue.Height) > _maxClimbHeight)
						continue;

					int nCost = _integrationGrid[nIdx];
					if (nCost < lowestCost)
					{
						lowestCost = nCost;
						bestNeighborPos = neighbor;
					}
				}

				if (bestNeighborPos != pos)
				{
					Vector2 dir2d = new Vector2(bestNeighborPos.X - pos.X, bestNeighborPos.Y - pos.Y).Normalized();
					_flowGrid[currentIdx] = new Vector3(dir2d.X, 0, dir2d.Y);
				}
				else
				{
					_flowGrid[currentIdx] = Vector3.Zero;
				}
			}
		}
	}

	public Vector3 GetFlowAtWorldPos(Vector3 worldPos)
	{
		Vector2I gridPos = _gridManager.WorldToGrid(worldPos);
		int index = GetLocalIndex(gridPos);
		
		if (index >= 0 && index < _flowGrid.Length)
			return _flowGrid[index];
			
		return Vector3.Zero;
	}

	public Rect2I GetFlowFieldBounds(Vector2I targetPos, List<Vector2I> unitPositions, int padding = 2)
	{
		int minX = targetPos.X;
		int maxX = targetPos.X;
		int minY = targetPos.Y;
		int maxY = targetPos.Y;

		foreach (Vector2I unitPos in unitPositions)
		{
			minX = Math.Min(minX, unitPos.X);
			maxX = Math.Max(maxX, unitPos.X);
			minY = Math.Min(minY, unitPos.Y);
			maxY = Math.Max(maxY, unitPos.Y);
		}

		minX -= padding;
		minY -= padding;
		maxX += padding;
		maxY += padding;

		// Clamp to GridManager global limits
		minX = Math.Max(minX, _gridManager.MapSizeMin.X);
		minY = Math.Max(minY, _gridManager.MapSizeMin.Y);
		maxX = Math.Min(maxX, _gridManager.MapSizeMax.X);
		maxY = Math.Min(maxY, _gridManager.MapSizeMax.Y);

		return new Rect2I(minX, minY, maxX - minX, maxY - minY);
	}

	private List<Vector2I> Get8NeighborsGlobal(Vector2I pos)
	{
		List<Vector2I> neighbors = new List<Vector2I>();
		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				if (x == 0 && y == 0) continue;

				Vector2I checkPos = pos + new Vector2I(x, y);
				if (IsWithinBounds(checkPos))
				{
					neighbors.Add(checkPos);
				}
			}
		}
		return neighbors;
	}

	private bool IsWithinBounds(Vector2I gridPos)
	{
		return gridPos.X >= _bounds.Position.X && gridPos.X < _bounds.End.X &&
		       gridPos.Y >= _bounds.Position.Y && gridPos.Y < _bounds.End.Y;
	}

	public int GetLocalIndex(Vector2I globalPos)
	{
		if (!IsWithinBounds(globalPos)) return -1;
		
		int localX = globalPos.X - _bounds.Position.X;
		int localY = globalPos.Y - _bounds.Position.Y;
		return localX + (localY * _bounds.Size.X);
	}
}
