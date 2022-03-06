using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

/*

    ToDo:
    - Ступеньки +-
    - Карабканье +-
    - Наклон камеры при беге по стене +-
        * плавный переход

    - Подкаты
    - Переписать метод движения (move)
        * Переосмыслить струкутру состояний
        * Переосмыслить функцию расчета скорости
    - Переосмыслить состояние in-air
    - Двойной прыжок +

    ToDo fix:
    - Скорость +
    - Застревание в стенах

    refactoring:
    описать стейты +
    подписать input на функции определения стейтов +
    подписать transition стейтов на функции transition +
    подписать performe стейтов на функции performe +


*/
public class MovementController : MonoBehaviour
{

    [Header("Objects inputs")]
    public Transform CharTF;
    private Transform CharGravityTF;
    public Transform CameraTF;
    public Rigidbody CharRB;
    public Collider CharCol;
    
    InputControll CharIS;


    [Header("Look parameters;")]
    private Vector2 lookInputVector;
    bool isCharRotating = false;
    bool isCameraRotating = false;
    public float lookSensetivity = 0.1f;

    [Header("Movement parameters:")]
    private Vector2 moveInputVector;
    public float maxRunSpeed = 2.7f; //in metrs per second
    public float accelerationTime = 1f; //in seconds
    [SerializeField]
    private Vector3 directionVector;
    [SerializeField]
    private Vector3 movementVector;
    public Vector3 speedMultiplier = new Vector3(0.9f, 0f, 0.7f);
    public float slopeSpeedMultiplier = 0.9f;
    [SerializeField]
    private float accelerationMoment = 0f;
    [SerializeField]
    private float currentSpeed = 0f;
    [SerializeField]
    public float inAirSpeed = 0.2f;
    public float maxRunSpeedLimit = 3f;

    [Header("CheckIsGrounded parameters")]
    public float groundMaxAngle = 40;
    [SerializeField]
    private float groundAngle;
    [SerializeField]
    Vector3 groundNormal;
    [SerializeField]
    public GameObject groundColliderObj;
    private Vector3 groundPos;
    //public Transform pivotGroundCheck;
    //public float groundCheckRadius = 0.2f;
    public LayerMask groundMask = 7;
    float defaultDrag;

    [Header("Jump parameters:")]
    public float jumpForce = 1f;
    public float jumpRecoil = 2f;

    [Header("Double jump parameters:")]
    [SerializeField]
    public Vector3 doubleJumpAddForceVector = new Vector3(0f, 1f, 0f);
    public float doubleJumpVectorMultiplier = 1f;

    [Header("Sprint parameters:")]
    public float sprintMaxSpeed = 2.7f;
    public float sprintAccelerationTime = 1f;
    public float sprintSpeedLimit = 7f;

    [Header("CheckWall parameters:")]
    public float CheckWallMinAngle = 44f;
    public float CheckWallMaxAngle = 91f;
    [SerializeField]
    public Vector3 CheckWallNormal = Vector3.zero;
    public GameObject CheckWallColliderObj;
    private Vector3 CheckWallClosestPoint;
    public float CheckWallAngle = 0f;

    [Header("Wall Running parameters:")]

    public float wallRunMinSpeed = 1f;
    public float wallRunMaxSpeed = 2.7f;
    public float wallRunAccelerationTime = 1f;
    public float wallRunCheckRadius = 0.2f;
    public float maxWallRunSpeedLimit = 12f;
    [SerializeField]
    public bool isWallRunState = false;
    
    [SerializeField]
    public float wallJumpVectorMultiplier = 1f;
    public Vector3 wallJumpAddForceVector = new Vector3(0f, 0f, 0f);

    [Header("CheckStepUp:")]
    private RaycastHit stepPointInfo;
    private float stepHeight;
    public float stepRation = 0.7f;

    private void Awake()
    {
        CharIS = new InputControll();
        CharGravityTF = this.GetComponentInParent<Transform>();
        defaultDrag = this.GetComponent<Rigidbody>().drag;

        CharIS.Player.MoveActionJump.started += _ => InputBeginJump();
        //CharIS.Player.MoveActionJump.performed += _ => ;
        //CharIS.Player.MoveActionJump.canceled += _ => ;

        //CharIS.Player.MoveActionJump.performed += _ => StartClimbing();
        //CharIS.Player.MoveActionJump.canceled += _ => StopClimbing();

        CharIS.Player.MoveActionSprint.performed += _ => InputPerformSprint();
        CharIS.Player.MoveActionSprint.canceled += _ => InputCancelSprint();

        //CharIS.Player.Fire.performed += _ => Shoot();

        CharIS.Player.Move.started += _ => InputBeginMove();
        CharIS.Player.Move.performed += _ => InputPerformMove(_.ReadValue<Vector2>());//StartMove(_.ReadValue<Vector2>());
        CharIS.Player.Move.canceled += _ => InputCancelMove();//StopMove();

        CharIS.Player.Look.performed += _ => StartCharRotate(_.ReadValue<Vector2>());
        CharIS.Player.Look.canceled += _ => StopCharRotate();

        CharIS.Player.Look.performed += _ => StartCameraRotate(_.ReadValue<Vector2>());
        CharIS.Player.Look.canceled += _ => StopCameraRotate();

        InitStates();
    }
    private void FixedUpdate()
    {
        State _tempCurState;
        if(CharCurrentState != (_tempCurState = CharCurrentState.InvokePerform())) //Continiously checks current state and changes for correct, if environment changes
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = CharCurrentState.InvokeTransition(_tempCurState);
        }

        RotateCharacter(lookInputVector);
        RotateCamera(lookInputVector);

        /* for refactoring
        if (jumpCheckStateNextTime < Time.realtimeSinceStartup)
        {
            //grounded params
            _tempIsGroundedState = isGrounded(groundColliders);
            if (!_tempIsGroundedState && isGroundedState)
            {
                wallRunCheckStateNextTime = Time.realtimeSinceStartup + wallRunCheckStateBlockTime;
            }

            isGroundedState = _tempIsGroundedState;

            //wall run params
            if (wallRunCheckStateNextTime < Time.realtimeSinceStartup)
            {
                _tempIsWallRunState = isWallRunning(wallRunColliders);
                if (!_tempIsWallRunState && isWallRunState)
                {
                    wallRunCheckStateNextTime = Time.realtimeSinceStartup + wallRunCheckStateBlockTime;
                }

                isWallRunState = _tempIsWallRunState;
            }
            
            //wall climbing params
            if(isClimbingInput)
            {
                _tempWallClimbingState = ((Vector3.zero != (_tempClimbUpPos = getClimbUpPosition(CameraTF, (CapsuleCollider)CharCol, CharTF))) ? true : false);//isClimable(wallClimbColliders);
                if (_tempWallClimbingState)
                    Climb();
                if (!_tempWallClimbingState && isWallClimbingState)
                {
                    wallRunCheckStateNextTime = Time.realtimeSinceStartup + wallRunCheckStateBlockTime;
                }
                isWallClimbingState = _tempWallClimbingState;
            }
                
            
        }

        if (isGroundedState)
        {
            if (inAirState) // is he in air after jump?
            {
                inAirState = false;
                CharRB.drag = defaultDrag;
                nextJumpTime = Time.realtimeSinceStartup + jumpRecoil;
            }
        }
        else
        {
            inAirState = true;
            CharRB.drag = 0;
        }
        Jump();
        Move(moveInputVector);
        */
    }

    private void Update()
    {

        



    }

    //States assign
    State Idle;
    State Fall;
    State Run;
    State Jump;
    State Sprint;
    State WallRun;
    State Climb;

    [Header("State info:")]
    [SerializeField]
    State CharCurrentState;
    [SerializeField]
    State CharPreviousState;
    [SerializeField]
    State CharPrePreviousState;

    void InitStates()
    {
        Fall = new State(new List<State>() { Idle, Jump, WallRun, Climb });
        Fall.SetPerform(new State.performMethods(PerformFall));
        Fall.SetTransition(Idle, new State.transitionMethods(CheckAndIdle));
        Fall.SetTransition(Jump, new State.transitionMethods(CheckAndDoubleJump));
        Fall.SetTransition(Climb, new State.transitionMethods(CheckAndClimb));
        Fall.SetTransition(WallRun, new State.transitionMethods(CheckAndWallRun));

        Idle = new State(new List<State>() { Fall, Run, Jump });
        Idle.SetPerform(new State.performMethods(PerformIdle));
        //Transition Idle-Fall is being called in perform method
        Idle.SetTransition(Run, new State.transitionMethods(CheckAndRun));
        Idle.SetTransition(Jump, new State.transitionMethods(CheckAndJump));

        Run = new State(new List<State>() { Fall, Idle, Sprint, Jump });
        Run.SetPerform(new State.performMethods(PerformRun));
        //Transition Run-Fall is being called in perform method
        Run.SetTransition(Idle, new State.transitionMethods(CheckAndIdle));
        Run.SetTransition(Sprint, new State.transitionMethods(CheckAndSprint));
        Run.SetTransition(Jump, new State.transitionMethods(CheckAndJump));

        Sprint = new State(new List<State>() { Fall, Run, Jump });
        Sprint.SetPerform(new State.performMethods(PerformSprint));
        //Transition Sprint-Fall is being called in perform method
        //Sprint.SetTransition(Idle, new State.transitionMethods(CheckAndIdle));
        Sprint.SetTransition(Run, new State.transitionMethods(CheckAndRun));
        Sprint.SetTransition(Jump, new State.transitionMethods(CheckAndJump));

        Jump = new State(new List<State>() { Fall });
        Jump.SetPerform(new State.performMethods(PerformJump));
        //Transition Jump-Fall is being called in transition Jump-* method


        WallRun = new State(new List<State>() { Fall, Jump, Climb });
        WallRun.SetPerform(new State.performMethods(PerformWallRun));
        //Transition WallRun-Fall is being called in perform method
        WallRun.SetTransition(Jump, new State.transitionMethods(CheckAndWallJump));
        WallRun.SetTransition(Climb, new State.transitionMethods(CheckAndClimb));

        //Repair climb methods?

        Climb = new State(new List<State>() { Fall, Jump, WallRun });
        Climb.SetPerform(new State.performMethods(PerformClimb));
        //Transition Climb-Fall is being called in perform method
        Climb.SetTransition(Jump, new State.transitionMethods(CheckAndWallJump));
        Climb.SetTransition(WallRun, new State.transitionMethods(CheckAndWallRun));
    }

    //State perfrom methods (being called every update/fixed update)

    State PerformFall()
    {
        if (CheckIsGrounded(groundColliderObj.GetComponent<GetCollidersList>().memCol))
            return Idle;
        else
            return CharCurrentState;
    }

    State PerformIdle()
    {
        if (CheckIsGrounded(groundColliderObj.GetComponent<GetCollidersList>().memCol))
            return CharCurrentState;
        else
            return Fall;
    }

    State PerformJump()
    {
        return Fall;
    }

    State PerformWallRun()
    {
        if (CheckWall(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol) && CheckCameraDirection(15f, 165f))
        {
            moveWallRun(moveInputVector, CheckWallNormal, wallRunMaxSpeed, wallRunAccelerationTime, maxWallRunSpeedLimit);
            return CharCurrentState;
        }
        else
            return Fall;
    }

    State PerformSprint()
    {
        if (CheckIsGrounded(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol))
        {
            moveRun(moveInputVector, groundNormal, sprintMaxSpeed, accelerationTime, sprintSpeedLimit);
            return CharCurrentState;
        }
        else
            return Fall;
    }

    State PerformRun()
    {
        if (CheckIsGrounded(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol))
        {
            moveRun(moveInputVector, groundNormal, maxRunSpeed, accelerationTime, maxRunSpeedLimit);
            return CharCurrentState;
        }
        else
            return Fall;
    }

    State PerformClimb()
    {
        if (CheckWall(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol) && CheckCameraDirection(150.1f, 181f))
        {
            moveClimb();
            return CharCurrentState;
        }
        else
            return Fall;
    }

    //Called by input methods (being called on input)

    void InputPerformSprint()
    {
        State _tempCurState;
        if (CharCurrentState != (_tempCurState = CharCurrentState.InvokeTransition(Sprint)))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCurState;
        }
    }

    void InputCancelSprint()
    {
        State _tempCurState;
        if (CharCurrentState != (_tempCurState = CharCurrentState.InvokeTransition(Run)))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCurState;
        }
    }

    void InputBeginJump()//transition to? Jump or WallRun or Climb
    {
        State _tempCharState;
        if (CharCurrentState != (_tempCharState = CharCurrentState.InvokeTransition(WallRun))) 
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
            return;
        };
        if (!(CharCurrentState == (_tempCharState = CharCurrentState.InvokeTransition(Climb))))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
            return;
        }
        if (CharCurrentState != (_tempCharState = CharCurrentState.InvokeTransition(Jump)))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
        }
    }

    void InputBeginMove()
    {
        State _tempCharState;
        if (!(CharCurrentState == (_tempCharState = CharCurrentState.InvokeTransition(WallRun))))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
            return;
        };

        if (!(CharCurrentState == (_tempCharState = CharCurrentState.InvokeTransition(Run))))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
        };
    }

    void InputCancelMove()
    {
        State _tempCharState;
        if (!(CharCurrentState == (_tempCharState = CharCurrentState.InvokeTransition(Idle))))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
            return;
        };
        if (!(CharCurrentState == (_tempCharState = CharCurrentState.InvokeTransition(Fall))))
        {
            CharPrePreviousState = CharPreviousState;
            CharPreviousState = CharCurrentState;
            CharCurrentState = _tempCharState;
            return;
        };
    }

    void InputPerformMove(Vector2 _moveInputVector)
    {
        moveInputVector = _moveInputVector;
    }

    //Check for correct transitions methods (being called on invoke.transition)

    bool CheckAndIdle()
    {
        return true;
    }

    bool CheckAndSprint()
    {
        return true;
    }

    bool CheckAndDoubleJump()
    {
        if (CharPreviousState == Jump && CharPrePreviousState != Fall) //DoubleJump
        {
            doDirectionalJump(jumpForce, doubleJumpVectorMultiplier, doubleJumpAddForceVector, false);
            return true;
        }
        return false;
    }

    bool CheckAndWallJump()
    {
        if (CheckWall(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol) && CheckCameraDirection(0f, 80f)) //WallRun jump
        {
            doDirectionalJump(jumpForce, wallJumpVectorMultiplier, wallJumpAddForceVector, true);
            return true;
        }
        return false;
    }

    bool CheckAndJump() //Basic jump
    {
        if(CheckIsGrounded(groundColliderObj.GetComponent<GetCollidersList>().memCol))
        {
            doBasicJump(jumpForce); //Regular Jump
            return true;
        }
        return false; 
    }

    bool CheckAndRun()
    {
        if (CheckIsGrounded(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol))
        {
            return Run;
        }
        else
            return Fall;
    }

    bool CheckAndClimb()
    {
        if (CheckWall(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol) && CheckCameraDirection(150.1f, 180f))
        {
            return true;
        }
        return false;
    }

    bool CheckAndWallRun()
    {
        if(CheckWall(CheckWallColliderObj.GetComponent<GetCollidersList>().memCol) && CheckCameraDirection(15f, 165f))
        {
            return true;
        }
        return false;
    }

    //Check methods (being used for calcucating correct next state)
    bool CheckIsGrounded(List<Collider> colliders)
    {
        bool _flagGroundFound = false;
        if (colliders.Count > 0)
        {
            //Get data about closest collider by raycasting to each

            Vector3 _origin = groundColliderObj.transform.position;
            Vector3 _closestPoint;
            float _minDistance = (Physics.ClosestPoint(_origin, colliders[0], colliders[0].gameObject.transform.position, colliders[0].transform.rotation) - _origin).magnitude;

            foreach (Collider _collider in colliders)
            {
                _closestPoint = Physics.ClosestPoint(_origin, _collider, _collider.gameObject.transform.position, _collider.transform.rotation);
                if ((_closestPoint - _origin).magnitude <= _minDistance)
                {
                    float _maxDistance = (_closestPoint - _origin).magnitude + 0.1f;
                    if (_collider.Raycast(new Ray(_origin, _closestPoint - _origin), out RaycastHit _hitInfo, _maxDistance))
                    {
                        float _tempGroundAngle = Mathf.Abs(Vector3.SignedAngle(CharGravityTF.up, _hitInfo.normal, Vector3.ProjectOnPlane(CharTF.forward, _hitInfo.normal)));
                        if (_tempGroundAngle < groundMaxAngle)
                        {
                            _minDistance = (_closestPoint - _origin).magnitude;
                            _flagGroundFound = true;
                            groundAngle = _tempGroundAngle;
                            groundNormal = _hitInfo.normal;
                            groundPos = _hitInfo.point;
                        }
                    }
                }
            }
        }
        else
        {
            groundPos = Vector3.zero;
            groundNormal = Vector3.zero;
            groundAngle = 0f;
            _flagGroundFound = false;
        }
        return _flagGroundFound;
    }

    bool CheckWall(List<Collider> colliders)
    {
        bool _flagWallFound = false;

        if (colliders.Count > 0)
        {
            //Get data about closest collider by raycasting to each

            Vector3 _origin = CheckWallColliderObj.transform.position;
            RaycastHit _hitInfo;
            Vector3 _closestPoint;
            float _minDistance = (Physics.ClosestPoint(_origin, colliders[0], colliders[0].gameObject.transform.position, colliders[0].transform.rotation) - _origin).magnitude;//get first collider distance

            foreach (Collider _collider in colliders)
            {
                _closestPoint = Physics.ClosestPoint(_origin, _collider, _collider.gameObject.transform.position, _collider.transform.rotation);//calculate closest point of current collider
                float _maxDistance = (_closestPoint - _origin).magnitude + 0.1f;//calculate distance to this point + 0.1f (for edges)
                if (_collider.Raycast(new Ray(_origin, _closestPoint - _origin), out _hitInfo, _maxDistance))//if raycast for this collider
                {
                    float _tempCheckWallAngle = Vector3.SignedAngle(CharGravityTF.up, _hitInfo.normal, Vector3.ProjectOnPlane(CharTF.forward, _hitInfo.normal));//get angle of closest plane

                    //bool _isMinSpeed = Vector3.ProjectOnPlane(CharRB.velocity, _hitInfo.normal).magnitude > wallRunMinSpeed;

                    if ((_closestPoint - _origin).magnitude <= _minDistance && Mathf.Abs(_tempCheckWallAngle) < CheckWallMaxAngle && Mathf.Abs(_tempCheckWallAngle) > CheckWallMinAngle)//distance <= then previous && angle is less then max-angle && higher then min-angle
                    {
                        CheckWallClosestPoint = _closestPoint;
                        _flagWallFound = true;
                        CheckWallAngle = _tempCheckWallAngle;
                        CheckWallNormal = _hitInfo.normal;
                        /*if (_isMinSpeed)
                        {
                            if (isWallRunState && wallRunNormal == _hitInfo.normal)
                            {
                                wallRunClosestPoint = _closestPoint;
                                _flagWallRunFound = true;
                            }
                            else
                            {
                                _flagWallRunFound = true;
                                wallRunClosestPoint = _closestPoint;
                                wallRunAngle = _tempWallRunAngle;
                                wallRunNormal = _hitInfo.normal;
                            }

                        }*/
                    }
                }
            }
        }
        else
        {
            CheckWallClosestPoint = Vector3.zero;
            //CharRB.useGravity = true;
            CheckWallNormal = Vector3.zero;
            CheckWallAngle = 0f;
            _flagWallFound = false;
        }
        return _flagWallFound;
    }

    bool CheckCameraDirection(float _minAngle, float _maxAngle) //does it calculates as I suppose?
    {
        float _tempAngle = Vector3.Angle(CameraTF.forward, CheckWallNormal);
        if (_minAngle <_tempAngle && _tempAngle < _maxAngle)
        {
            return true;
        }
        return false;
    }

    //Perform methods (being called )

    void doDirectionalJump(float _jumpForce, float _jumpVectorMultiplier, Vector3 _additionalJumpForceVector, bool _flagStopOnJump) //double jump or wall jump
    {
        CharRB.drag = 0f;
        Vector3 _jumpVector = (Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, CharGravityTF.up), CharGravityTF.up) * new Vector3(moveInputVector.x, 0, moveInputVector.y)).normalized + _additionalJumpForceVector;
        _jumpVector.x = _jumpVector.x * _jumpVectorMultiplier;
        _jumpVector.z = _jumpVector.z * _jumpVectorMultiplier;

        if(_flagStopOnJump) CharRB.velocity = Vector3.zero;

        CharRB.velocity += _jumpVector * _jumpForce / CharRB.mass;
    }

    void doBasicJump(float _jumpForce) //basic jump
    {
        CharRB.drag = 0f;
        Vector3 _jumpVector = CharGravityTF.up;
        CharRB.velocity += _jumpVector * _jumpForce / CharRB.mass;
    }

    void moveRun(Vector2 _moveInput, Vector3 _groundNormal, float _maxSpeed, float _accelerationTime, float _maxSpeedLimit)
    {
        float _prevVelocity;

        directionVector = Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, _groundNormal).normalized, CharGravityTF.up) * new Vector3(_moveInput.x, 0, _moveInput.y); //calculate desired direction vector (Projects on to ground normal and scaled by moveInput)
        if (Mathf.Abs(Vector3.SignedAngle(CharRB.velocity, directionVector, Vector3.Cross(CharRB.velocity, directionVector))) < 90f) //velocity is being dumped by character rotation
        {
            _prevVelocity = Vector3.Project(CharRB.velocity, directionVector).magnitude;
        }
        else
        {
            _prevVelocity = 0f;
        }

        if (!CharRB.useGravity) //in case if after wallrun or climb gravity was not turned back on
            CharRB.useGravity = false;

        //acceleration moment and speed is being recalculated in case of change state (for example wallrun -> run or sprint -> run)

        accelerationMoment = accelerationMoment > 0.01f ? CalcNewAccelMoment(_prevVelocity, _maxSpeed, _accelerationTime) + Time.deltaTime : Time.deltaTime; 

        if (_prevVelocity <= _maxSpeed)
            currentSpeed = CalcSpeed(_maxSpeed, _accelerationTime, accelerationMoment);
        else
        if (_prevVelocity > _maxSpeedLimit)
            currentSpeed = _prevVelocity * slopeSpeedMultiplier;
        else
            currentSpeed = _prevVelocity;

        if (CharRB.drag == defaultDrag) //drag is being turned off at run/sprint/wallrun state
            CharRB.drag = 0f;

        //Additional logic for managing steps
        Vector3 _stepVelocity = Vector3.zero;

        if (CheckStepUp())
        {
            Vector3 _stepForceDir = CharGravityTF.up;
            bool _midStep = stepHeight > stepRation * (groundColliderObj.GetComponent<SphereCollider>().radius * 2f);
            if (_midStep)
            {
                //Debug.Log("Mid Step");
                float _stepPower = (stepHeight + 0.2f) * 5f;
                _stepVelocity = _stepForceDir * _stepPower;
            }

            else
            {
                //Debug.Log("Low Step");
                float _stepPower = (stepHeight + 0.2f) * 4f;
                _stepVelocity = _stepForceDir * _stepPower;
            }
        }

        //CharCol.material.frictionCombine = PhysicMaterialCombine.Minimum;
        CharRB.velocity = directionVector * currentSpeed + _stepVelocity;
    }

    void moveWallRun(Vector2 _moveInput, Vector3 _wallNormal, float _maxSpeed, float _accelerationTime, float _maxSpeedLimit)
    {
        float _prevVelocity;
        Vector3 temp = CheckWallAngle < 0 ? Vector3.Cross(_wallNormal, CharGravityTF.up * 1f).normalized : Vector3.Cross(_wallNormal, CharGravityTF.up * -1f).normalized;
        _moveInput.x = 0f;
        if (_moveInput.y < 0f)
            _moveInput.y = 0f;
        directionVector = Quaternion.LookRotation(temp, CharGravityTF.up) * new Vector3(_moveInput.x, 0, _moveInput.y);
        float _tempDirectionAngle = Vector3.SignedAngle(CharRB.velocity, CharTF.forward, CheckWallAngle < 0 ? CharGravityTF.up : CharGravityTF.up * -1f);
        if (_tempDirectionAngle > -45f && _tempDirectionAngle < 135f)
        {
            _prevVelocity = CharRB.velocity.magnitude;
        }
        else
        {
            _prevVelocity = 0f;
        }
        if (CharRB.useGravity)
            CharRB.useGravity = false;

        //acceleration moment and speed is being recalculated in case of change state (for example wallrun -> run or sprint -> run)

        accelerationMoment = accelerationMoment > 0.01f ? CalcNewAccelMoment(_prevVelocity, _maxSpeed, _accelerationTime) + Time.deltaTime : Time.deltaTime;

        if (_prevVelocity <= _maxSpeed)
            currentSpeed = CalcSpeed(_maxSpeed, _accelerationTime, accelerationMoment);
        else
        if (_prevVelocity > _maxSpeedLimit)
            currentSpeed = _prevVelocity * slopeSpeedMultiplier;
        else
            currentSpeed = _prevVelocity;

        if (CharRB.drag == defaultDrag) //drag is being turned off at run/sprint/wallrun state
            CharRB.drag = 0f;

        //Additional logic for managing steps
        Vector3 _stepVelocity = Vector3.zero;

        if (CheckStepUp())
        {
            Vector3 _stepForceDir = CharGravityTF.up;
            bool _midStep = stepHeight > stepRation * (groundColliderObj.GetComponent<SphereCollider>().radius * 2f);
            if (_midStep)
            {
                //Debug.Log("Mid Step");
                float _stepPower = (stepHeight + 0.2f) * 5f;
                _stepVelocity = _stepForceDir * _stepPower;
            }

            else
            {
                //Debug.Log("Low Step");
                float _stepPower = (stepHeight + 0.2f) * 4f;
                _stepVelocity = _stepForceDir * _stepPower;
            }
        }

        //CharCol.material.frictionCombine = PhysicMaterialCombine.Minimum;
        CharRB.velocity = directionVector * currentSpeed + _stepVelocity;

        //end
    }

    void moveClimb()
    {

    }

    //old methods (should be refactored?)

    void StartCharRotate(Vector2 inputVector)
    {
        if (!inputVector.Equals(lookInputVector))
            lookInputVector = inputVector;
        isCharRotating = true;
    }

    void StopCharRotate()
    {
        lookInputVector = Vector2.zero;
        isCharRotating = false;
    }

    void RotateCharacter(Vector2 viewInput)
    {
        if (isCharRotating)
        {
            CharTF.localEulerAngles = new Vector3(CharTF.localEulerAngles.x, CameraTF.eulerAngles.y, CharTF.localEulerAngles.z);
        }
    }

    void StartCameraRotate(Vector2 inputVector)
    {
        if (!inputVector.Equals(lookInputVector))
            lookInputVector = inputVector;
        isCameraRotating = true;
    }

    void StopCameraRotate()
    {
        lookInputVector = Vector2.zero;
        isCameraRotating = false;
    }

    void RotateCamera(Vector2 viewInput)
    {
        if (isCameraRotating)
        {
            float rotationY = CameraTF.localEulerAngles.y + viewInput.x * lookSensetivity;
            float rotationX = CameraTF.localEulerAngles.x + viewInput.y * lookSensetivity * -1;
            rotationX = rotationX > 180 && rotationX <= 270 ? 269.9f : rotationX >= 90 && rotationX < 180 ? 89.9f : rotationX;
            float rotationZ = isWallRunState ? (CheckWallAngle > 0f ? 7f : -7f) : CharGravityTF.localEulerAngles.z;
            CameraTF.localEulerAngles = new Vector3(rotationX, rotationY, rotationZ);
        }
    }

    private float CalcSpeed(float _maxSpeed, float _accelerationTime, float _accelerationMoment)
    {
        float _constantK = _maxSpeed * 2f / 3.14f;
        return (float)System.Math.Round(_constantK * Mathf.Atan(5f / _accelerationTime * _accelerationMoment), 2);
    }

    private float CalcNewAccelMoment(float _speed, float _newMaxSpeed, float _newAccelerationTime)
    {
        float _constantK = _newMaxSpeed * 2f / 3.14f;
        return _newAccelerationTime / 5f * Mathf.Abs(Mathf.Tan(_speed / _constantK));
    }

    bool CheckStepUp()
    {
        stepPointInfo = new RaycastHit();
        RaycastHit _hitInfoChecker;
        float _rayDistance = groundColliderObj.GetComponent<SphereCollider>().radius * 2f;
        Vector3 _originLowest = groundColliderObj.transform.position + CharGravityTF.up * groundColliderObj.GetComponent<SphereCollider>().radius * -0.8f;

        //sends few raycasts for identifying ability to step up
        if (Physics.Raycast(_originLowest, directionVector, out _hitInfoChecker, _rayDistance, groundMask) && Vector3.Angle(directionVector, _hitInfoChecker.normal) - 90 > groundMaxAngle) 
        {
            //Debug.Log("Required Step!");
            RaycastHit _hitInfoHigh;
            Vector3 _origin = groundColliderObj.transform.position + (CharGravityTF.up * groundColliderObj.GetComponent<SphereCollider>().radius * 1f);
            if (!Physics.Raycast(_origin, directionVector, out _hitInfoHigh, _hitInfoChecker.distance + 0.05f, groundMask))
            {
                _origin += directionVector.normalized * _hitInfoChecker.distance;
                if (Physics.Raycast(_origin, CharGravityTF.up * -1f, out _hitInfoHigh, _rayDistance, groundMask))
                {
                    stepPointInfo = _hitInfoHigh;
                    stepHeight = Vector3.Project((stepPointInfo.point - groundPos), CharGravityTF.up).magnitude;
                    return true;
                }
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;
    }
    
    //old code for refactor
    
    /*
    private void OnEnable()
    {
        CharIS.Enable();
    }

    private void OnDisable()
    {
        CharIS.Disable();
    }

    void StartJump()
    {
        isJumpingState = true;
    }

    void StopJump()
    {
        isJumpingState = false;
    }

    bool canJump()
    {

        if(Time.realtimeSinceStartup > nextJumpTime)
        {
            if (isGroundedState)
            {
                return true;
            }

            if (isWallRunState)
            {
                return true;
            }

            if (isWallClimbingState)
                return true;

            if (inAirState)
            {
                return false;
            }

            if (isDoubleJumpingState)
            {
                return false;
            }
        }
        return false;
    }

    //refactor jump method
    void Jump()
    {
        if (isJumpingState && !isWallClimbingState)
        {
            if (canDoubleJump())
            {
                CharRB.drag = 0f;
                isDoubleJumpingState = true;
                Vector3 doubleJumpVector = (Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, CharGravityTF.up), CharGravityTF.up) * new Vector3(moveInputVector.x, 0, moveInputVector.y)).normalized + doubleJumpAddForceVector;//Vector3.Angle(CharRB.velocity, CameraTF.forward) < 90f ? Vector3.Project(CharRB.velocity, CameraTF.forward) : -1 * Vector3.Project(CharRB.velocity, CameraTF.forward);
                doubleJumpVector.x = doubleJumpVector.x * doubleJumpVectorMultiplier;
                doubleJumpVector.z = doubleJumpVector.z * doubleJumpVectorMultiplier;

                CharRB.velocity += doubleJumpVector * jumpForce / CharRB.mass;
                jumpCheckStateNextTime = Time.realtimeSinceStartup + jumpCheckStateBlockTime;
                isGroundedState = false;
                StopJump();
            }

            if (canJump())
                if (isWallRunState)
                {
                    CharRB.drag = 0f;
                    Vector3 wallJumpVector = (wallRunNormal * 0f + Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, CharGravityTF.up), CharGravityTF.up) * new Vector3(moveInputVector.x, 0, moveInputVector.y)).normalized + wallJumpAddForceVector;//Vector3.Project(CharRB.velocity, CameraTF.forward).magnitude < 1f ? Vector3.Project(CharRB.velocity, CameraTF.forward).normalized : Vector3.Project(CharRB.velocity, CameraTF.forward);
                    wallJumpVector.x = wallJumpVector.x * wallJumpVectorMultiplier;
                    wallJumpVector.z = wallJumpVector.z * wallJumpVectorMultiplier;

                    if (Vector3.Angle(CameraTF.forward, wallRunNormal) < 80f)
                    {
                        CharRB.velocity = Vector3.zero;
                        CharRB.velocity = wallJumpVector * jumpForce / CharRB.mass;
                        isWallRunState = false;
                        jumpCheckStateNextTime = Time.realtimeSinceStartup + jumpCheckStateBlockTime;
                        StopJump();
                    }
                }
                else
            if (isWallClimbingState)
                {

                    Vector3 wallJumpVector = (wallRunNormal * 0f + Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, CharGravityTF.up), CharGravityTF.up) * new Vector3(moveInputVector.x, 0, moveInputVector.y)).normalized + wallJumpAddForceVector;//Vector3.Project(CharRB.velocity, CameraTF.forward).magnitude < 1f ? Vector3.Project(CharRB.velocity, CameraTF.forward).normalized : Vector3.Project(CharRB.velocity, CameraTF.forward);
                    wallJumpVector.x = wallJumpVector.x * wallJumpVectorMultiplier;
                    wallJumpVector.z = wallJumpVector.z * wallJumpVectorMultiplier;
                    if (Vector3.Angle(CameraTF.forward, wallRunNormal) < 80f)
                    {
                        CharRB.drag = 0f;
                        CharRB.velocity = Vector3.zero;
                        CharRB.velocity = wallJumpVector * jumpForce / CharRB.mass;
                        isWallClimbingState = false;
                        jumpCheckStateNextTime = Time.realtimeSinceStartup + jumpCheckStateBlockTime;
                        StopJump();
                    }
                }
                else
                {
                    CharRB.drag = 0f;
                    Vector3 jumpVector = CameraTF.forward * 0f + CharGravityTF.up;
                    CharRB.velocity += jumpVector * jumpForce / CharRB.mass;
                    isGroundedState = false;
                    jumpCheckStateNextTime = Time.realtimeSinceStartup + jumpCheckStateBlockTime;
                    StopJump();
                }
        }

    }

    void StartSprint()
    {
        isSprintingState = true;
    }

    void StopSprint()
    {
        isSprintingState = false;
    }

    void Shoot()
    {
        Debug.Log("Shot fired!");
    }

    bool isWallRunning(List<Collider> colliders)
    {

        if (isGroundedState) return false;
        if (isWallClimbingState) return false;
        if (!isMoving) return false;
        
        bool flagWallRunFound = false;

        if (colliders.Count > 0)
        {
            //Get data about closest collider by raycasting to each

            Vector3 _origin = wallRunCollidersObj.transform.position;
            RaycastHit _hitInfo;
            Vector3 _closestPoint;
            float _minDistance = (Physics.ClosestPoint(_origin, colliders[0], colliders[0].gameObject.transform.position, colliders[0].transform.rotation) - _origin).magnitude;//get first collider distance

            foreach (Collider _collider in colliders)
            {
                _closestPoint = Physics.ClosestPoint(_origin, _collider, _collider.gameObject.transform.position, _collider.transform.rotation);//calculate closest point of current collider
                float _maxDistance = (_closestPoint - _origin).magnitude + 0.1f;//calculate distance to this point + 0.1f (for edges)
                if (_collider.Raycast(new Ray(_origin, _closestPoint - _origin), out _hitInfo, _maxDistance))//if raycast for this collider
                { 
                    float _tempWallRunAngle = Vector3.SignedAngle(CharGravityTF.up, _hitInfo.normal, Vector3.ProjectOnPlane(CharTF.forward, _hitInfo.normal));//get angle of closest plane
                    bool _isMinSpeed = Vector3.ProjectOnPlane(CharRB.velocity, _hitInfo.normal).magnitude > wallRunMinSpeed;
                    if ((_closestPoint - _origin).magnitude <= _minDistance && Mathf.Abs(_tempWallRunAngle) < wallRunMaxAngle && Mathf.Abs(_tempWallRunAngle) > wallRunMinAngle)//distance <= then previous and angle is less then max-angle
                    {
                        if (_isMinSpeed)
                        {
                            if (isWallRunState && wallRunNormal == _hitInfo.normal) 
                            {
                                wallRunClosestPoint = _closestPoint;
                                flagWallRunFound = true;
                            }
                            else
                            {
                                flagWallRunFound = true;
                                wallRunClosestPoint = _closestPoint;
                                wallRunAngle = _tempWallRunAngle;
                                wallRunNormal = _hitInfo.normal;
                            }

                        }
                    }
                }
            }
        }
        else
        {
            wallRunClosestPoint = Vector3.zero;
            CharRB.useGravity = true;
            wallRunNormal = Vector3.zero;
            wallRunAngle = 0f;
            flagWallRunFound = false;
        }
        return flagWallRunFound;
    }

    void StartMove (Vector2 inputVector)
    {
        if (!inputVector.Equals(moveInputVector))
            moveInputVector = inputVector;
        isMoving = true;
    }

    void StopMove()
    {
        isMoving = false;
        moveInputVector = Vector2.zero;
        accelerationMoment = 0f;
        movementVector = Vector2.zero;
    }

    //refactor Move method
    void Move (Vector2 moveInput)
    {
        
        if (isMoving && (isGroundedState || isWallRunState || isWallClimbingState))
        {
            float maxSpeed;
            float accelTime;
            float prevVelocity;
            float maxSpeedLimit;

            if (isWallRunState)
            {
                Vector3 temp = wallRunAngle < 0 ? Vector3.Cross(wallRunNormal, CharGravityTF.up * 1f).normalized : Vector3.Cross(wallRunNormal, CharGravityTF.up * -1f).normalized;
                moveInput.x = 0f;
                if (moveInput.y < 0f)
                    moveInput.y = 0f;
                directionVector = Quaternion.LookRotation(temp, CharGravityTF.up) * new Vector3(moveInput.x, 0, moveInput.y);
                maxSpeed = wallRunMaxSpeed;
                accelTime = wallRunAccelerationTime;
                maxSpeedLimit = maxWallRunSpeedLimit;
                float _tempDirectionAngle = Vector3.SignedAngle(CharRB.velocity, CharTF.forward, wallRunAngle < 0 ? CharGravityTF.up : CharGravityTF.up * -1f);
                if (_tempDirectionAngle > -45f && _tempDirectionAngle < 135f)
                {
                    prevVelocity = CharRB.velocity.magnitude; 
                }
                else
                {
                    prevVelocity = 0f;
                }
                if (CharRB.useGravity)
                    CharRB.useGravity = false;
            }
            else
            if (isWallClimbingState && false)//remove
            {
                Vector3 temp = Vector3.ProjectOnPlane(CharGravityTF.up, wallClimbNormal).normalized;
                directionVector = Quaternion.LookRotation(temp, wallClimbNormal) * new Vector3(moveInput.x, 0, moveInput.y);
                maxSpeed = wallClimbingMaxSpeed;
                accelTime = wallClimbingAccelerationTime;
                maxSpeedLimit = wallClimbingMaxSpeed;
                prevVelocity = 0f;
                if (CharRB.useGravity)
                    CharRB.useGravity = false;
            }
            else
            if (isSprintingState)
            {
                directionVector = Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, groundNormal).normalized, groundNormal) * new Vector3(moveInput.x, 0, moveInput.y);
                maxSpeed = sprintMaxSpeed;
                accelTime = sprintAccelerationTime;
                maxSpeedLimit = maxSprintSpeedLimit;
                if (Mathf.Abs(Vector3.SignedAngle(CharRB.velocity, directionVector, Vector3.Cross(CharRB.velocity, directionVector))) < 90f )
                {
                    prevVelocity = Vector3.Project(CharRB.velocity, directionVector).magnitude;
                }
                else
                {
                    prevVelocity = 0f;
                }
                if (!CharRB.useGravity)
                    CharRB.useGravity = true;
            }
            else
            if (inAirState)
            {
                directionVector = Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, CharGravityTF.up).normalized, CharGravityTF.up) * new Vector3(moveInput.x, 0, moveInput.y);
                maxSpeed = 0f;
                accelTime = 1f;
                maxSpeedLimit = 0f;
                if (Mathf.Abs(Vector3.SignedAngle(CharRB.velocity, directionVector, Vector3.Cross(CharRB.velocity, directionVector))) < 90f)
                {
                    prevVelocity = Vector3.Project(CharRB.velocity, directionVector).magnitude;
                }
                else
                {
                    prevVelocity = 0f;
                }
                if (!CharRB.useGravity)
                    CharRB.useGravity = true;
            }
            else
            {
                directionVector = Quaternion.LookRotation(Vector3.ProjectOnPlane(CharTF.forward, groundNormal).normalized, CharGravityTF.up) * new Vector3(moveInput.x, 0, moveInput.y);
                maxSpeed = maxRunSpeed;
                accelTime = accelerationTime;
                maxSpeedLimit = maxRunSpeedLimit;
                if (Mathf.Abs(Vector3.SignedAngle(CharRB.velocity, directionVector, Vector3.Cross(CharRB.velocity, directionVector))) < 90f)
                {
                    prevVelocity = Vector3.Project(CharRB.velocity, directionVector).magnitude;
                }
                else
                {
                    prevVelocity = 0f;
                }
                if (!CharRB.useGravity)
                    CharRB.useGravity = false;
            }

            accelerationMoment = accelerationMoment > 0.01f ? CalcNewAccelMoment(prevVelocity, maxSpeed, accelTime) + Time.deltaTime : Time.deltaTime;

            if (prevVelocity <= maxSpeed)
                currentSpeed = CalcSpeed(maxSpeed, accelTime, accelerationMoment);
            else
            if (prevVelocity > maxSpeedLimit)
                currentSpeed = prevVelocity * slopeSpeedMultiplier;
            else
                currentSpeed = prevVelocity;

            if (CharRB.drag == defaultDrag)
                CharRB.drag = 0f;


            Vector3 _stepVelocity = Vector3.zero;
            bool _tempCanStepUp = canStepUp();
            
            if (_tempCanStepUp)
            {
                
                
                Vector3 _stepForceDir = CharGravityTF.up;
                bool _midStep = stepHeight > stepRation * (groundCollidersObj.GetComponent<SphereCollider>().radius * 2f);
                if (_midStep)
                {
                    Debug.Log("Mid Step");
                    float _stepPower = (stepHeight + 0.2f) * 5f;
                    _stepVelocity = _stepForceDir * _stepPower;
                }

                else
                {
                    Debug.Log("Low Step");
                    float _stepPower = (stepHeight + 0.2f) * 4f;
                    _stepVelocity = _stepForceDir * _stepPower;
                }
            }

            CharCol.material.frictionCombine = PhysicMaterialCombine.Minimum;
            CharRB.velocity = directionVector * currentSpeed + _stepVelocity;

        }
        else
        {
            if (!inAirState && CharRB.drag != defaultDrag)
                CharRB.drag = defaultDrag;
            CharRB.useGravity = true;
            currentSpeed = 0f;
            if (isGroundedState) CharCol.material.frictionCombine = PhysicMaterialCombine.Maximum;
        }

    }

    void StartCharRotate(Vector2 inputVector)
    {
        if (!inputVector.Equals(lookInputVector))
            lookInputVector = inputVector;
        isCharRotating = true;
    }

    void StopCharRotate()
    {
        lookInputVector = Vector2.zero;
        isCharRotating = false;
    }

    void RotateCharacter (Vector2 viewInput)
    {
        if (isCharRotating)
        {
            CharTF.localEulerAngles = new Vector3(CharTF.localEulerAngles.x, CameraTF.eulerAngles.y, CharTF.localEulerAngles.z);
        }
    }

    void StartCameraRotate(Vector2 inputVector)
    {
        if (!inputVector.Equals(lookInputVector))
            lookInputVector = inputVector;
        isCameraRotating = true;
    }

    void StopCameraRotate()
    {
        lookInputVector = Vector2.zero;
        isCameraRotating = false;
    }

    void RotateCamera(Vector2 viewInput)
    {
        if (isCameraRotating)
        {
            float rotationY = CameraTF.localEulerAngles.y + viewInput.x * lookSensetivity;
            float rotationX = CameraTF.localEulerAngles.x + viewInput.y * lookSensetivity * -1;
            rotationX = rotationX > 180 && rotationX <= 270 ? 269.9f : rotationX >= 90 && rotationX < 180 ? 89.9f : rotationX;
            float rotationZ = isWallRunState ? (wallRunAngle > 0f ? 7f : -7f) : CharGravityTF.localEulerAngles.z;
            CameraTF.localEulerAngles = new Vector3(rotationX, rotationY, rotationZ);
        }
    }

    private bool isGrounded(List<Collider> colliders)
    {
        
        bool flagGroundFound = isGroundedState;
        if (colliders.Count > 0)
        {
            //Get data about closest collider by raycasting to each

            Vector3 _origin = groundCollidersObj.transform.position;
            RaycastHit _hitInfo;
            Vector3 _closestPoint;
            float _minDistance = (Physics.ClosestPoint(_origin, colliders[0], colliders[0].gameObject.transform.position, colliders[0].transform.rotation) - _origin).magnitude;

            foreach (Collider _collider in colliders)
            {
                _closestPoint = Physics.ClosestPoint(_origin, _collider, _collider.gameObject.transform.position, _collider.transform.rotation);
                if ((_closestPoint - _origin).magnitude <= _minDistance)
                {
                    float _maxDistance = (_closestPoint - _origin).magnitude + 0.1f;
                    if (_collider.Raycast(new Ray(_origin, _closestPoint - _origin), out _hitInfo, _maxDistance))
                    {
                        float _tempGroundAngle = Mathf.Abs(Vector3.SignedAngle(CharGravityTF.up, _hitInfo.normal, Vector3.ProjectOnPlane(CharTF.forward, _hitInfo.normal)));
                        if (_tempGroundAngle < maxGroundedAngle)
                        {
                            _minDistance = (_closestPoint - _origin).magnitude;
                            flagGroundFound = true;
                            groundAngle = _tempGroundAngle;
                            groundNormal = _hitInfo.normal;
                            groundedPosition = _hitInfo.point;
                        }
                    }
                }
            }
        }
        else
        {
            groundedPosition = Vector3.zero;
            groundNormal = Vector3.zero;
            groundAngle = 0f;
            flagGroundFound = false;
        }
        return flagGroundFound;
    }

    Vector3 _tempClimbUpPos;

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        if (_tempWallClimbingState)
        {
            Gizmos.DrawWireSphere(_tempClimbUpPos, 0.1f);
        }
        //Gizmos.color = Color.green;
        //Gizmos.DrawLine(debugClosestPoint, debugClosestPoint + debugVector1);
        //Gizmos.color = Color.blue;
        //Gizmos.DrawLine(debugClosestPoint, debugClosestPoint + debugVector2);
    }

    private void StartClimbing()
    {
        isClimbingInput = true;
    }

    private void StopClimbing()
    {
        isClimbingInput = false;
    }

    bool canDoubleJump()
    {
        if (isGroundedState)
        {
            isDoubleJumpingState = false;
            return false;
        }

        if (isWallRunState)
        {
            isDoubleJumpingState = false;
            return false;
        }

        if (isWallClimbingState)
        {
            isDoubleJumpingState = false;
            return false;
        }

        if (isDoubleJumpingState)
        {
            isDoubleJumpingState = true;
            return false;
        }

        if (inAirState)
        { 
            return true; 
        }
            

        return false;
    }

    bool canStepUp() 
    {
        stepPointInfo = new RaycastHit();
        RaycastHit _hitInfoChecker;
        float _rayDistance = groundCollidersObj.GetComponent<SphereCollider>().radius * 2f;
        Vector3 _originLowest = pivotGroundCheck.position + CharGravityTF.up * groundCollidersObj.GetComponent<SphereCollider>().radius * -0.8f;

        if (Physics.Raycast(_originLowest, directionVector, out _hitInfoChecker, _rayDistance, groundMask) && Vector3.Angle(directionVector, _hitInfoChecker.normal) - 90 > maxGroundedAngle)
        {
            //Debug.Log("Required Step!");
            RaycastHit _hitInfoHigh;
            Vector3 _origin = pivotGroundCheck.position + (CharGravityTF.up * groundCollidersObj.GetComponent<SphereCollider>().radius * 1f);
            if (!Physics.Raycast(_origin, directionVector, out _hitInfoHigh, _hitInfoChecker.distance + 0.05f, groundMask))
            {
                _origin += directionVector.normalized * _hitInfoChecker.distance;
                if (Physics.Raycast(_origin, CharGravityTF.up * -1f, out _hitInfoHigh, _rayDistance, groundMask))
                {
                    stepPointInfo = _hitInfoHigh;
                    stepHeight = Vector3.Project((stepPointInfo.point - groundedPosition), CharGravityTF.up).magnitude;
                    return true;
                }
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;
    }


    [Header("Climb parameters:")]
    [SerializeField]
    private bool isClimbingInput;
    public bool isWallClimbingState = false;
    private float wallClimbingMaxSpeed = 1f;
    private float wallClimbingAccelerationTime = 0.1f;
    private Vector3 wallClimbNormal;
    public GameObject wallClimbCollidersObj;


    Vector3 getClimbUpPosition(Transform camera, CapsuleCollider characterCollider, Transform characterTransform)
    {
        float _deltaAngle = 0f;
        float _epsilonAngle = 1f;
        float _handleDistance = characterCollider.height / 3f;
        float _epsilonDistance = _handleDistance * 0.1f;
        Vector3 _checkRayPivot = camera.position;
        
        Vector3 _checkRayOrigin;
        float _angle;
        Vector3 _checkRayDirection = Vector3.zero;
        Ray _checkRay;
        RaycastHit _raycastHit;
        float _rayMaxDistance = _handleDistance;

        for (int i = 1; i <= 2; i += 1)
        {
            do
        {
            for (float _tempCurrentDistance = 0f; _tempCurrentDistance <= _handleDistance; _tempCurrentDistance += _epsilonDistance)
            {
                    _checkRayDirection = Quaternion.AngleAxis(_deltaAngle * (i == 1 ? -1 : 1), camera.right) * camera.forward;
                    if (!Physics.Raycast(_checkRayPivot, _checkRayDirection, out _raycastHit, _tempCurrentDistance, groundMask))
                    {
                        _checkRayOrigin = _checkRayPivot + _checkRayDirection * _tempCurrentDistance;
                        _checkRay = new Ray(_checkRayOrigin, characterTransform.up * -1);

                        if (Physics.Raycast(_checkRay, out _raycastHit, _rayMaxDistance, groundMask))
                        {
                            //projectCharacterCollider if 
                            if (!Physics.CheckCapsule(_raycastHit.point + characterTransform.up * characterCollider.radius, _raycastHit.point + characterTransform.up * characterCollider.height, characterCollider.radius * 0.8f, groundMask))//Physics.SphereCast(_raycastHit.point, characterCollider.radius, characterTransform.up, out _tempHit, characterCollider.height, groundMask))
                            {
                                Debug.Log("success");
                                return _raycastHit.point;
                            }
                        }
                    }
                }
                _angle = Vector3.Angle(camera.forward, _checkRayDirection);
                _deltaAngle += _epsilonAngle;
            }
            while (_angle < 30) ;//-45;45
        }

        return Vector3.zero;
    }
    
    void Climb()
    {
        CharTF.position = _tempClimbUpPos + CharTF.up * 0.5f;
    }
    */
}
