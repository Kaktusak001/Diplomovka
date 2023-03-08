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
    [SerializeField] private float lineSubDivision;
    [SerializeField] private float angleAdjust;
    [SerializeField] private float distanceAdjust;

    public List<Command> commands = new List<Command>();
    private bool runningCommand;

    private void Start()
    {
        GoTo(-2f, 10f);
        /*
        commands.Add(RotAngle(15f * Mathf.Deg2Rad, null));
        commands.Add(GoForward(4.5f, null));
        for (int i = 0; i < 100; i++)
        {
            commands.Add(RotAngle(5f * Mathf.Deg2Rad, null));
            commands.Add(GoForward(0.01f, null));
            commands.Add(RotAngle(-4f * Mathf.Deg2Rad, null));
        }*/
    }

    public bool GetCommandStatus => runningCommand;

    public void UpdateNavigation()
    {
        Vector3 estimatedPosition = roomba.GetEstimatedPosition();
        float estimatedRotation = roomba.GetEstimatedRotation();
        
        GetCellsInRange(new Vector2(estimatedPosition.x, estimatedPosition.z), sweepWidth / 2f, true).ForEach(c => grid[c.x].celly[c.y].visited = true);

        Sensor sensorData = roomba.GetSensorData(0);

        Vector3 startPos = estimatedPosition + Quaternion.Euler(0f, estimatedRotation, 0f) * sensorData.Location;
        Vector3 position = estimatedPosition + Quaternion.Euler(0f, estimatedRotation, 0f) * (sensorData.Location + Vector3.right * (sensorData.Distance < 0f ? roomba.GetSensorMaxDistance : sensorData.Distance));

        GetCellsOnLine(new Vector2(startPos.x, startPos.z), new Vector2(position.x, position.z), lineSubDivision, true);

        if (sensorData.Distance >= 0f)
        {
            Vector2Int location = new Vector2Int((int) (position.x / cellSize), (int) (position.z / cellSize));

            GetGridCell(location, out Vector2Int coords, true);
            grid[coords.x].celly[coords.y].wall = true;
        }

        if (runningCommand)
        {
            if (roomba.rightPower > 0f && roomba.leftPower > 0f && (roomba.GetLeftBumperData() || roomba.GetRightBumperData() || roomba.DetectCliff()))
            {
                runningCommand = false;
                StopCommand(-1, true); // change to -1
            }
            else if (commands[0].time > 0f)
                commands[0].time -= Time.fixedDeltaTime;
            else
            {
                runningCommand = false;
                StopCommand(0, false);
            }
        }
        else if (commands.Count > 0)
        {
            roomba.rightPower = commands[0].rightPower;
            roomba.leftPower = commands[0].leftPower;
            runningCommand = true;
        }
    }

    public Command GoForward(float distance, Action<bool> invoke) => new Command(1f, 1f, roomba.DistToTime(distance), invoke);
    public Command RotAngle(float angle, Action<bool> invoke) => angle < 0f ? new Command(1f, -1f, -roomba.RotToTime(angle), invoke) : new Command(-1f, 1f, roomba.RotToTime(angle), invoke);

    private Vector3 currentGoTo;
    public void GoTo(float x, float z)
    {
        currentGoTo = new Vector3(x, 0f, z);
        Vector3 direction = new Vector3(x, 0f, z) - roomba.GetEstimatedPosition();
        float angle = Quaternion.FromToRotation(Vector3.forward, direction).eulerAngles.y - roomba.GetEstimatedRotation();
        angle = angle % 360f;
        if (angle > 180f)
            angle -= 360f;
        
        commands.Add(RotAngle(angle * Mathf.Deg2Rad, null));
        commands.Add(GoForward(direction.magnitude, ContinueGoTo));
    }

    public void ResumeGoTo(bool premature)
    {
        if (premature)
            ContinueGoTo(true);
        else
            GoTo(currentGoTo.x, currentGoTo.z);
    }

    private void ContinueGoTo(bool premature)
    {
        if (!premature)
            return;

        float angle = angleAdjust * Mathf.Deg2Rad;
        bool isRightCliff = false;

        if (roomba.DetectCliff(out List<bool> sensorActivation))
        {
            for (int i = (int) (sensorActivation.Count / 2f); i < sensorActivation.Count; i++)
            {
                if (sensorActivation[i])
                {
                    isRightCliff = true;
                    break;
                }
            }
        }

        if (roomba.GetRightBumperData() || isRightCliff)
            angle = -angle;
        
        commands.Insert(0, GoForward(distanceAdjust, ResumeGoTo));
        commands.Insert(0, RotAngle(angle, null));
    }

    public void StopCommand(int index, bool premature)
    {
        Action<bool> function = commands[0].invoke;

        if (index == -1)
            commands.Clear();
        else if (index > commands.Count - 1)
            Debug.LogError("Invalid command index !!!");
        else
            commands.RemoveAt(index);

        roomba.rightPower = 0f;
        roomba.leftPower = 0f;

        function?.Invoke(premature);
    }

    public List<Vector2Int> GetCellsOnLine(Vector2 start, Vector2 end, float subDivision, bool create)
    {
        int count = (int) ((end - start).magnitude / subDivision);
        Vector2 add = (end - start).normalized * subDivision;
        Vector2 point = start;

        List<Vector2Int> newCells = new List<Vector2Int>();
        Vector2Int newCell;
        Vector2Int cellPos = new Vector2Int((int) (point.x / cellSize), (int) (point.y / cellSize));
        Vector2Int lastCellPos = -cellPos;
        
        for (int i = 0; i <= count; i++)
        {
            if (lastCellPos != cellPos)
                if (GetGridCell(cellPos, out newCell, create) || create)
                    newCells.Add(newCell);

            lastCellPos = cellPos;
            point += add;
            cellPos = new Vector2Int((int) (point.x / cellSize), (int) (point.y / cellSize));
        }

        if (GetGridCell(new Vector2Int((int) (end.x / cellSize), (int) (end.y / cellSize)), out newCell, create) || create)
            newCells.Add(newCell);

        return newCells;
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
                
                if (!create)
                    return false;
                
                grid[x].celly.Add(new CellY(indexes.y, false, false));
                coords = new Vector2Int(x, grid[x].celly.Count - 1);
                return false;
            }

            if (grid[x].index <= indexes.x)
                continue;
            
            if (!create)
                return false;
            
            grid.Insert(x, new CellX(indexes.x, new List<CellY>() {new CellY(indexes.y, false, false)}));
            coords = new Vector2Int(x, 0);
            return false;
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

[Serializable]
public class Command
{
    public float rightPower;
    public float leftPower;
    public float time;
    public Action<bool> invoke;

    public Command(float rightPower, float leftPower, float time, Action<bool> invoke)
    {
        this.rightPower = rightPower;
        this.leftPower = leftPower;
        this.time = time;
        this.invoke = invoke;
    }
}