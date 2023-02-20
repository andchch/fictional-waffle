using System;
using System.Text.RegularExpressions;
using System.Collections;
using System.IO.Ports;
using System.Security.Permissions;
using UnityEngine;

public class Main : MonoBehaviour
{
    Vector3 _position;
    Vector3 _rotation;
    public Vector3 rotationOffset ;
    public float speedFactor = 15.0f;
    public bool euler = false;
    
    public GameObject fal1;
    public GameObject fal2;
    public GameObject fal3;
    public GameObject fal4;
    
    Regex qrgx = new Regex(@"-?\d?\d_-?\d?\d_-?\d?\d_-?\d?\d_-?0,\d\d\d\d_-?0,\d\d\d\d_-?0,\d\d\d\d_-?0,\d\d\d\d");
    Regex ergx = new Regex(@"-?\d\d_-?\d\d_-?\d\d_-?\d\d_-?\d?\d?\d,\d\d_-?\d?\d?\d,\d\d_-?\d?\d?");
    
    private int ang1, ang2, ang3, ang4;
    private float qw, qx, qy, qz;
    
    private float e1, e2, e3;
    private Vector3 currentEulerAngles;

    private string _data;
    SerialPort port;
    
    void Start()
    {
        port = new SerialPort("COM4", 115200);
        port.Open();
        
        ang1 = -1;
        ang2 = -1;
        ang3 = -1;
        ang4 = -1;
        
        qw = 0.1f;
        qx = 0.1f;
        qy = 0.1f;
        qz = 0.1f;
        
        e1 = 90f;
        e2 = 90f;
        e3 = 90f;

        currentEulerAngles = new Vector3(0, 0, 0);
    }
    
    void Update()
    {
        
        StartCoroutine(Delay());
        //fal1.transform.localRotation = new Quaternion(0, 0, ang1, 90);
        //fal2.transform.localRotation = new Quaternion(0, 0, ang2, 90);
        //fal3.transform.localRotation = new Quaternion(0, 0, ang3, 90);
        //fal4.transform.localRotation = new Quaternion(0, 0, ang4, 90);

        if (euler)
        {
            //currentEulerAngles += new Vector3(e1, e2, e3) * Time.deltaTime * speedFactor;
            transform.localRotation = Quaternion.Slerp(transform.localRotation, 
                Quaternion.Euler(e1,e2,e3), Time.deltaTime * speedFactor);
        }
        else
        {
            transform.localRotation = Quaternion.Lerp(transform.localRotation, 
                new Quaternion(qw, qy, qx, qz), Time.deltaTime * speedFactor);
        }
        transform.parent.transform.eulerAngles = rotationOffset;
    }

    bool IsDataCorrect(string input)
    {
        bool isCorrect = true;
        if (euler)
        {
            if (!ergx.IsMatch(input) || input.Length > 47)
                isCorrect = false; 
            return isCorrect;
        }
        else
        {
            if (!qrgx.IsMatch(input) || input.Length > 47)
                isCorrect = false; 
            return isCorrect;
        }
    }

    IEnumerator Delay()
    {
        _data = port.ReadLine();
        _data = _data.Replace('.', ',');
        Debug.Log(_data);

        if (IsDataCorrect(_data))
        {
            if (euler)
            {
                string[] values = _data.Split('_');
                
                ang1 = int.Parse(values[0]);
                ang2 = int.Parse(values[1]);
                ang3 = int.Parse(values[2]);
                ang4 = int.Parse(values[3]);
                
                e1 = float.Parse(values[4]);
                e2 = float.Parse(values[5]);
                e3 = float.Parse(values[6]);
            }
            else
            {
                string[] values = _data.Split('_');

                ang1 = int.Parse(values[0]);
                ang2 = int.Parse(values[1]);
                ang3 = int.Parse(values[2]);
                ang4 = int.Parse(values[3]);

                qw = float.Parse(values[4]);
                qx = float.Parse(values[5]);
                qy = float.Parse(values[6]);
                qz = float.Parse(values[7]);
            }
        }
        else
        {
            Debug.Log("Invalid data: "+ _data);
        }
        yield return new WaitForSeconds(0.3f);
    }

    public void OnClick()
    {
        port.Write("start calibration");
    }

    void OnApplicationQuit()
    {
        port.Close();
    }
}
