using System;
using System.Collections;
using System.Collections.Generic;
using System.Dynamic;
using System.Linq;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ConsoleScript : MonoBehaviour
{
    [SerializeField] private GameObject console;
    
    [SerializeField] private TMP_Text consoleText;
    [SerializeField] private int maxLines;
    
    [SerializeField] private TMP_InputField consoleInput;

    [SerializeField] private RoombaScript roomba;
    [SerializeField] private NavigationScript navigation;

    public void OnConsoleInput()
    {
        string input = consoleInput.text;
        consoleInput.text = "";
        if (input == "")
            return;

        Command(input);
    }

    public void Command(string command)
    {
        PrintLine(command);
        string[] parts = command.Replace("\n", string.Empty).ToLower().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        
        float float0;
        float float1;

        switch (parts.Count())
        {
            case 1:
                switch (parts[0])
                {
                    case "/reset":
                        roomba.ResetRoomba();
                        break;
                    case "/calibrate":
                        navigation.Calibrate();
                        break;
                    case "/stop":
                        navigation.Stop();
                        break;
                    case "/sensors":
                        string tmp = "";
                        roomba.GetSensorData().ForEach(s => tmp += ", " + s.Distance);
                        tmp = tmp.Remove(0, 1);
                        PrintLine("Distance sensors:" + tmp);
                        tmp = "";
                        roomba.GetCliffSensorData().ForEach(s => tmp += ", " + s.Distance);
                        tmp = tmp.Remove(0, 1);
                        PrintLine("Cliff sensors:" + tmp);
                        PrintLine("Bumpers: " + roomba.GetLeftBumperData() + ", " + roomba.GetRightBumperData());
                        break;
                    default:
                        PrintLine("Invalid command!");
                        break;
                }
                break;
            case 2:
                switch (parts[0])
                {
                    case "/forward":
                        if (float.TryParse(parts[1], out float0) && float0 > 0f)
                            navigation.Forward(float0, 1f);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/backward":
                        if (float.TryParse(parts[1], out float0) && float0 > 0f)
                            navigation.Backward(float0, 1f);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/right":
                        if (float.TryParse(parts[1], out float0) && float0 is > 0f or <= 180f)
                            navigation.Right(float0, 1f);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/left":
                        if (float.TryParse(parts[1], out float0) && float0 is > 0f or <= 180f)
                            navigation.Left(float0, 1f);
                        else
                            PrintLine("Invalid command!");
                        break;
                    default:
                        PrintLine("Invalid command!");
                        break;
                }
                break;
            case 3:
                switch (parts[0])
                {
                    case "/forward":
                        if (float.TryParse(parts[1], out float0) && float.TryParse(parts[2], out float1) && float0 > 0f && float1 is > 0f or <= 1f)
                            navigation.Forward(float0, float1);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/backward":
                        if (float.TryParse(parts[1], out float0) && float.TryParse(parts[2], out float1) && float0 > 0f && float1 is > 0f or <= 1f)
                            navigation.Backward(float0, float1);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/right":
                        if (float.TryParse(parts[1], out float0) && float.TryParse(parts[2], out float1) && float0 is > 0f or <= 180f && float1 is > 0f or <= 1f)
                            navigation.Right(float0, float1);
                        else
                            PrintLine("Invalid command!");
                        break;
                    case "/left":
                        if (float.TryParse(parts[1], out float0) && float.TryParse(parts[2], out float1) && float0 is > 0f or <= 180f && float1 is > 0f or <= 1f)
                            navigation.Left(float0, float1);
                        else
                            PrintLine("Invalid command!");
                        break;
                    default:
                        PrintLine("Invalid command!");
                        break;
                }
                break;
            default:
                PrintLine("Invalid command!");
                break;
        }
        
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.BackQuote))
            console.SetActive(!console.activeSelf);
    }

    public void PrintLine(string line)
    {
        List<string> lines = line.Split("\n", StringSplitOptions.RemoveEmptyEntries).ToList();
        
        while (lines.Count > maxLines)
            lines.RemoveAt(0);

        List<string> text = consoleText.text.Split("\n", StringSplitOptions.RemoveEmptyEntries).ToList();
        
        while (text.Count + lines.Count > maxLines)
            text.RemoveAt(0);

        consoleText.text = "";
        foreach (string t in text)
            consoleText.text += t + "\n";
        
        foreach (string t in lines)
            consoleText.text += t + "\n";
    }
}
