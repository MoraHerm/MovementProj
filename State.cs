using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// State creation example
//Idle = new State(new List<State>() {Run, Fall}); 

// Transition creation example
//State.transitionMethods IdleRun = null;
//Idle.SetTransition(Run, IdleRun);

// Perform creation example
//State.performMethods IdleRun = null;
//Idle.SetPerform(IdleRun);

// Invoke transition example
//Idle = Idle.InvokeTransition(Run);

// Invoke perform (for continious execute in update method)
//Idle.InvokePerform();

    //todo: specify exceptions(?)
public class State : MonoBehaviour
{
    public delegate bool transitionMethods();
    public delegate State performMethods();
    private performMethods Perform;

    private Dictionary<State, transitionMethods> transitions; //Use smthing else?
    
    public State(List<State> states)
    {
        foreach(State _state in states)
        {
            transitionMethods _transition = null;
            transitions.Add(_state, _transition);
        }
        Perform = null;
    }

    public void SetTransition (State _state, transitionMethods _transition)
    {
        if (this.transitions.TryGetValue(_state, out transitionMethods tempTrans))
        {
            transitions.Remove(_state);
            transitions.Add(_state, _transition);
        }
        else 
            throw new System.Exception("Transition for state \""+_state.ToString()+"\" is not allowed.");
    }

    public State InvokeTransition(State _state)//being called once at transition
    {
        if (this.transitions.TryGetValue(_state, out transitionMethods tempTrans))
        {
            if (tempTrans.Invoke())
                return _state;
            else
                return this;
        }
        else
            return this;
    }

    public void SetPerform (performMethods _performe)
    {
        Perform = _performe;
    }

    public State InvokePerform ()
    {
        if(Perform != null)
        {
            return Perform.Invoke();
        }
        else
            throw new System.Exception("Perform for state \"" + this.ToString() + "\" is not allowed.");
    }
}