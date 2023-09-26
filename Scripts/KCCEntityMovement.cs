using KinematicCharacterController;
using LiteNetLib.Utils;
using LiteNetLibManager;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace MultiplayerARPG
{
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(CapsuleCollider))]
    [RequireComponent(typeof(KinematicCharacterMotor))]
    public class KCCEntityMovement : BaseNetworkedGameEntityComponent<BaseGameEntity>, IEntityMovementComponent, ICharacterController, IBuiltInEntityMovement3D
    {
        [Header("Movement AI")]
        [Range(0.01f, 1f)]
        public float stoppingDistance = 0.1f;
        public MovementSecure movementSecure = MovementSecure.NotSecure;

        [Header("Movement Settings")]
        public bool allowJumpingWhenSliding = false;
        public float jumpHeight = 2f;
        public ApplyJumpForceMode applyJumpForceMode = ApplyJumpForceMode.ApplyImmediately;
        public float applyJumpForceFixedDuration;
        public float backwardMoveSpeedRate = 0.75f;
        public float gravity = 9.81f;
        public float maxFallVelocity = 40f;
        public LayerMask platformLayerMask = 1;
        [Tooltip("Delay before character change from grounded state to airborne")]
        public float airborneDelay = 0.01f;
        public bool doNotChangeVelocityWhileAirborne;
        public float landedPauseMovementDuration = 0f;
        public float beforeCrawlingPauseMovementDuration = 0f;
        public float afterCrawlingPauseMovementDuration = 0f;
        [Range(0.1f, 1f)]
        public float underWaterThreshold = 0.75f;
        public bool autoSwimToSurface;

        [Header("Ground checking")]
        public float forceUngroundAfterJumpDuration = 0.1f;

        [Header("Root Motion Settings")]
        public bool useRootMotionForMovement;
        public bool useRootMotionForAirMovement;
        public bool useRootMotionForJump;
        public bool useRootMotionForFall;
        public bool useRootMotionWhileNotMoving;
        public bool useRootMotionUnderWater;

        [Header("Networking Settings")]
        public float snapThreshold = 5.0f;

        public Animator CacheAnimator { get; private set; }
        public Rigidbody CacheRigidbody { get; private set; }
        public CapsuleCollider CacheCapsuleCollider { get; private set; }
        public KinematicCharacterMotor CacheMotor { get; private set; }
        public BuiltInEntityMovementFunctions3D Functions { get; private set; }

        public float StoppingDistance { get { return Functions.StoppingDistance; } }
        public MovementState MovementState { get { return Functions.MovementState; } }
        public ExtraMovementState ExtraMovementState { get { return Functions.ExtraMovementState; } }
        public DirectionVector2 Direction2D { get { return Functions.Direction2D; } set { Functions.Direction2D = value; } }
        public float CurrentMoveSpeed { get { return Functions.CurrentMoveSpeed; } }
        public Queue<Vector3> NavPaths { get { return Functions.NavPaths; } }
        public bool HasNavPaths { get { return Functions.HasNavPaths; } }

        protected Vector3 _motion;
        protected float _yAngle;
        protected Vector3 _internalVelocityAdd;

        public override void EntityAwake()
        {
            // Prepare animator component
            CacheAnimator = GetComponent<Animator>();
            // Prepare rigidbody component
            CacheRigidbody = gameObject.GetOrAddComponent<Rigidbody>();
            // Prepare collider component
            CacheCapsuleCollider = gameObject.GetOrAddComponent<CapsuleCollider>();
            // Prepare open character controller
            float radius = CacheCapsuleCollider.radius;
            float height = CacheCapsuleCollider.height;
            Vector3 center = CacheCapsuleCollider.center;
            CacheMotor = gameObject.GetOrAddComponent<KinematicCharacterMotor>();
            CacheMotor.CharacterController = this;
            // Disable unused component
            LiteNetLibTransform disablingComp = gameObject.GetComponent<LiteNetLibTransform>();
            if (disablingComp != null)
            {
                Logging.LogWarning(nameof(KCCEntityMovement), "You can remove `LiteNetLibTransform` component from game entity, it's not being used anymore [" + name + "]");
                disablingComp.enabled = false;
            }
            // Setup
            Functions = new BuiltInEntityMovementFunctions3D(Entity, CacheAnimator, this)
            {
                stoppingDistance = stoppingDistance,
                movementSecure = movementSecure,
                jumpHeight = jumpHeight,
                applyJumpForceMode = applyJumpForceMode,
                applyJumpForceFixedDuration = applyJumpForceFixedDuration,
                backwardMoveSpeedRate = backwardMoveSpeedRate,
                gravity = gravity,
                maxFallVelocity = maxFallVelocity,
                stickGroundForce = 0f,
                airborneDelay = airborneDelay,
                doNotChangeVelocityWhileAirborne = doNotChangeVelocityWhileAirborne,
                landedPauseMovementDuration = landedPauseMovementDuration,
                beforeCrawlingPauseMovementDuration = beforeCrawlingPauseMovementDuration,
                afterCrawlingPauseMovementDuration = afterCrawlingPauseMovementDuration,
                underWaterThreshold = underWaterThreshold,
                autoSwimToSurface = autoSwimToSurface,
                useRootMotionForMovement = useRootMotionForMovement,
                useRootMotionForAirMovement = useRootMotionForAirMovement,
                useRootMotionForJump = useRootMotionForJump,
                useRootMotionForFall = useRootMotionForFall,
                useRootMotionWhileNotMoving = useRootMotionWhileNotMoving,
                useRootMotionUnderWater = useRootMotionUnderWater,
                snapThreshold = snapThreshold,
            };
            Functions.StopMoveFunction();
        }

        public override void EntityStart()
        {
            Functions.EntityStart();
            CacheMotor.SetPosition(CacheTransform.position);
        }

        public override void ComponentOnEnable()
        {
            Functions.ComponentEnabled();
            CacheMotor.enabled = true;
            try
            {
                CacheMotor.SetPosition(CacheTransform.position);
            }
            catch { }
        }

        public override void ComponentOnDisable()
        {
            CacheMotor.enabled = false;
        }

        public override void OnSetOwnerClient(bool isOwnerClient)
        {
            base.OnSetOwnerClient(isOwnerClient);
            Functions.OnSetOwnerClient(isOwnerClient);
        }

        private void OnAnimatorMove()
        {
            Functions.OnAnimatorMove();
        }

        private void OnTriggerEnter(Collider other)
        {
            Functions.OnTriggerEnter(other);
        }

        private void OnTriggerExit(Collider other)
        {
            Functions.OnTriggerExit(other);
        }

        public void BeforeCharacterUpdate(float deltaTime)
        {
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {
            Functions.UpdateRotation(deltaTime);
            currentRotation = Quaternion.Euler(0, _yAngle, 0);
        }

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            Functions.UpdateMovement(deltaTime);
            currentVelocity = _motion / deltaTime;
        }

        public void AfterCharacterUpdate(float deltaTime)
        {
            Functions.AfterMovementUpdate(deltaTime);
        }

        public bool IsColliderValidForCollisions(Collider coll)
        {
            return true;
        }

        public void OnGroundHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
        {
            Functions.OnControllerColliderHit(hitPoint, hitCollider.transform);
        }

        public void OnMovementHit(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, ref HitStabilityReport hitStabilityReport)
        {
        }

        public void PostGroundingUpdate(float deltaTime)
        {
        }

        public void AddVelocity(Vector3 velocity)
        {
            _internalVelocityAdd += velocity;
        }

        public void ProcessHitStabilityReport(Collider hitCollider, Vector3 hitNormal, Vector3 hitPoint, Vector3 atCharacterPosition, Quaternion atCharacterRotation, ref HitStabilityReport hitStabilityReport)
        {
        }

        public void OnDiscreteCollisionDetected(Collider hitCollider)
        {
        }

        public bool GroundCheck()
        {
            return CacheMotor.GroundingStatus.IsStableOnGround;
        }

        public Bounds GetBounds()
        {
            return CacheCapsuleCollider.bounds;
        }

        public void Move(Vector3 motion)
        {
            _motion = motion;
        }

        public void RotateY(float yAngle)
        {
            _yAngle = yAngle;
        }

        public void OnJumpForceApplied(float verticalVelocity)
        {
            CacheMotor.ForceUnground(forceUngroundAfterJumpDuration);
        }

        public bool WriteClientState(long writeTimestamp, NetDataWriter writer, out bool shouldSendReliably)
        {
            return Functions.WriteClientState(writeTimestamp, writer, out shouldSendReliably);
        }

        public bool WriteServerState(long writeTimestamp, NetDataWriter writer, out bool shouldSendReliably)
        {
            return Functions.WriteServerState(writeTimestamp, writer, out shouldSendReliably);
        }

        public void ReadClientStateAtServer(long peerTimestamp, NetDataReader reader)
        {
            Functions.ReadClientStateAtServer(peerTimestamp, reader);
        }

        public void ReadServerStateAtClient(long peerTimestamp, NetDataReader reader)
        {
            Functions.ReadServerStateAtClient(peerTimestamp, reader);
        }

        public void StopMove()
        {
            Functions.StopMove();
        }

        public void KeyMovement(Vector3 moveDirection, MovementState movementState)
        {
            Functions.KeyMovement(moveDirection, movementState);
        }

        public void PointClickMovement(Vector3 position)
        {
            Functions.PointClickMovement(position);
        }

        public void SetExtraMovementState(ExtraMovementState extraMovementState)
        {
            Functions.SetExtraMovementState(extraMovementState);
        }

        public void SetLookRotation(Quaternion rotation)
        {
            Functions.SetLookRotation(rotation);
        }

        public Quaternion GetLookRotation()
        {
            return Functions.GetLookRotation();
        }

        public void SetSmoothTurnSpeed(float speed)
        {
            Functions.SetSmoothTurnSpeed(speed);
        }

        public float GetSmoothTurnSpeed()
        {
            return Functions.GetSmoothTurnSpeed();
        }

        public void Teleport(Vector3 position, Quaternion rotation, bool stillMoveAfterTeleport)
        {
            Functions.Teleport(position, rotation, stillMoveAfterTeleport);
        }

        public bool FindGroundedPosition(Vector3 fromPosition, float findDistance, out Vector3 result)
        {
            return Functions.FindGroundedPosition(fromPosition, findDistance, out result);
        }
    }
}
