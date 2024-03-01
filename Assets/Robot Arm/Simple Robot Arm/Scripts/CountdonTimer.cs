using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CountdownTimer : MonoBehaviour
{
    public float time;
    public bool isRunning = false;

    public delegate void TimeoutEvent();
    public TimeoutEvent timeoutEvent;

    /// <summary>
    /// Start the timer
    /// </summary>
    public void StartTimer(float newTime, TimeoutEvent timeoutEvent)
    {
        isRunning = true;
        time = newTime;
        this.timeoutEvent = timeoutEvent;
    }

    /// <summary>
    /// Reset the timer the timer
    /// </summary>
    public void ResetTimer()
    {
        isRunning = false;
        time = 0;
    }

    /// <summary>
    /// Decrease the timer by the time passed. If the timer reaches 0, call the timeout event
    /// </summary>
    public void DecreaseTimer()
    {
        if(time > 0)
        {
            time -= Time.deltaTime;
        } else {
            ResetTimer();
            timeoutEvent();
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(isRunning)
        {
            DecreaseTimer();
        }
        
    }
}
