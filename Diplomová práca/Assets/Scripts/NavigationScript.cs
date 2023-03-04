using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

public class NavigationScript : MonoBehaviour
{
    [SerializeField] private RoombaScript roomba;
    [SerializeField] private ConsoleScript console;

    public List<CellX> grid;
    [SerializeField] private float cellSize;
    [SerializeField] private float sweepWidth;

    public void UpdateNavigation()
    {
        Vector3 estimatedPosition = roomba.GetEstimatedPosition();
        float estimatedRotation = roomba.GetEstimatedRotation();
        
        GetCellsInRange(new Vector2(estimatedPosition.x, estimatedPosition.z), sweepWidth / 2f, true).ForEach(c => grid[c.x].celly[c.y].visited = true);

        Sensor sensorData = roomba.GetSensorData(0);
        
        if (sensorData.Distance < 0f)
            return;

        Vector3 position = estimatedPosition + Quaternion.Euler(0f, estimatedRotation, 0f) * (sensorData.Location + Vector3.right * sensorData.Distance);
        Vector2Int location = new Vector2Int((int) (position.x / cellSize), (int) (position.z / cellSize));

        GetGridCell(location, out Vector2Int coords, true);
        grid[coords.x].celly[coords.y].wall = true;
    }
    
    public List<Vector2Int> GetCellsInRange(Vector2 position, float range, bool create)
    {
        List<Vector2Int> cells = new List<Vector2Int>();

        Vector2Int startIndex = new Vector2Int(Mathf.FloorToInt((position.x - range) / cellSize), Mathf.FloorToInt((position.y - range) / cellSize));
        Vector2Int endIndex = new Vector2Int(Mathf.CeilToInt((position.x + range) / cellSize), Mathf.CeilToInt((position.y + range) / cellSize));

        Vector2Int tmpCoords;
        for (int x = startIndex.x; x <= endIndex.x; x++)
        {
            for (int y = startIndex.y; y <= endIndex.y; y++)
            {
                if ((position - new Vector2(x * cellSize, y * cellSize)).magnitude > range)
                    continue;
                if (GetGridCell(new Vector2Int(x, y), out tmpCoords, create) || create)
                    cells.Add(tmpCoords);
            }
        }
        
        return cells;
    }

    public List<Vector2Int> GetCellsRect(Vector2Int startIndex, Vector2Int endIndex, float range, bool create)
    {
        List<Vector2Int> cells = new List<Vector2Int>();

        int tmp;
        if (startIndex.x > endIndex.x)
        {
            tmp = startIndex.x;
            startIndex.x = endIndex.x;
            endIndex.x = tmp;
        }

        if (startIndex.y > endIndex.y)
        {
            tmp = startIndex.y;
            startIndex.y = endIndex.y;
            endIndex.y = tmp;
        }

        Vector2Int tmpCoords;
        for (int x = startIndex.x; x <= endIndex.x; x++)
        {
            for (int y = startIndex.y; y <= endIndex.y; y++)
            {
                if (GetGridCell(new Vector2Int(x, y), out tmpCoords, create) || create)
                    cells.Add(tmpCoords);
            }
        }
        
        return cells;
    }

    public bool GetGridCell(Vector2Int indexes, out Vector2Int coords, bool create) // TODO TRANSFER TO NAVIGATION
    {
        coords = Vector2Int.zero;
        
        for (int x = 0; x < grid.Count; x++)
        {
            if (grid[x].index == indexes.x)
            {
                for (int y = 0; y < grid[x].celly.Count; y++)
                {
                    if (grid[x].celly[y].index == indexes.y)
                    {
                        coords = new Vector2Int(x, y);
                        return true;
                    }
                    if (grid[x].celly[y].index <= indexes.y)
                        continue;
                    
                    if (!create)
                        return false;
                    
                    grid[x].celly.Insert(y, new CellY(indexes.y, false, false));
                    coords = new Vector2Int(x, y);
                    return false;
                }
            }
            else if (grid[x].index > indexes.x)
            {
                if (!create)
                    return false;
                
                grid.Insert(x, new CellX(indexes.x, new List<CellY>() {new CellY(indexes.y, false, false)}));
                coords = new Vector2Int(x, 0);
                return false;
            }
        }

        if (!create)
            return false;
        
        grid.Add( new CellX(indexes.x, new List<CellY>() {new CellY(indexes.y, false, false)}));
        coords = new Vector2Int(grid.Count - 1, 0);
        return false;
    }

    private void OnDrawGizmos()
    {
        if (roomba.GetDrawGizmos())
        {
            grid.ForEach(x => x.celly.ForEach(y =>
            {
                Gizmos.color = y.visited ? Color.white : Color.black;
                Gizmos.DrawRay(roomba.GetBaseStationOffset() + new Vector3(cellSize * x.index, 0f, cellSize * y.index), Vector3.up * 0.3f);
                
                if (!y.wall) return;
                
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(roomba.GetBaseStationOffset() + new Vector3(cellSize * x.index, 0.3f, cellSize * y.index), Vector3.up * 0.3f);
            }));
        }
    }
}