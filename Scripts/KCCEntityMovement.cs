using KinematicCharacterController;
using LiteNetLib.Utils;
using LiteNetLibManager;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace MultiplayerARPG
{
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(CapsuleCollider))]
    [RequireComponent(typeof(KinematicCharacterMotor))]
    public class KCCEntityMovement : BaseNetworkedGameEntityComponent<BaseGameEntity>, IEntityMovementComponent, ICharacterController, IBuiltInEntityMovement3D
    {
        private static readonly RaycastHit[] s_findGroundRaycastHits = new RaycastHit[8];

        [Header("Movement AI")]
        [Range(0.01f, 1f)]
        public float stoppingDistance = 0.1f;
        public MovementSecure movementSecure = MovementSecure.NotSecure;

        [Header("Movement Settings")]
        public float jumpHeight = 2f;
        public ApplyJumpForceMode applyJumpForceMode = ApplyJumpForceMode.ApplyImmediately;
        public float applyJumpForceFixedDuration;
        public float backwardMoveSpeedRate = 0.75f;
        public float gravity = 9.81f;
        public float maxFallVelocity = 40f;
        [Tooltip("Delay before character change from grounded state to airborne")]
        public float airborneDelay = 0.01f;
        public bool doNotChangeVelocityWhileAirborne;

        [Header("Pausing")]
        public float landedPauseMovementDuration = 0f;
        public float beforeCrawlingPauseMovementDuration = 0f;
        public float afterCrawlingPauseMovementDuration = 0f;

        [Header("Swimming")]
        [Range(0.1f, 1f)]
        public float underWaterThreshold = 0.75f;
        public bool autoSwimToSurface;

        [Header("Ground checking")]
        public float forceUngroundAfterJumpDuration = 0.1f;

        [Header("Dashing")]
        public EntityMovementForceApplierData dashingForceApplier = EntityMovementForceApplierData.CreateDefault();

        [Header("Root Motion Settings")]
        [FormerlySerializedAs("useRootMotionWhileNotMoving")]
        public bool alwaysUseRootMotion;
        public bool useRootMotionForMovement;
        public bool useRootMotionForAirMovement;
        public bool useRootMotionForJump;
        public bool useRootMotionForFall;
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
        protected Vector3 _internalVelocityAdd;

        public override void EntityAwake()
        {
            // Prepare animator component
            CacheAnimator = GetComponent<Animator>();
            if (CacheAnimator == null)
                CacheAnimator = GetComponentInChildren<Animator>();
            // Prepare rigidbody component
            CacheRigidbody = gameObject.GetOrAddComponent<Rigidbody>();
            CacheRigidbody.useGravity = false;
            // Prepare collider component
            CacheCapsuleCollider = gameObject.GetOrAddComponent<CapsuleCollider>();
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
                airborneDelay = airborneDelay,
                doNotChangeVelocityWhileAirborne = doNotChangeVelocityWhileAirborne,
                landedPauseMovementDuration = landedPauseMovementDuration,
                beforeCrawlingPauseMovementDuration = beforeCrawlingPauseMovementDuration,
                afterCrawlingPauseMovementDuration = afterCrawlingPauseMovementDuration,
                underWaterThreshold = underWaterThreshold,
                autoSwimToSurface = autoSwimToSurface,
                alwaysUseRootMotion = alwaysUseRootMotion,
                dashingForceApplier = dashingForceApplier,
                useRootMotionForMovement = useRootMotionForMovement,
                useRootMotionForAirMovement = useRootMotionForAirMovement,
                useRootMotionForJump = useRootMotionForJump,
                useRootMotionForFall = useRootMotionForFall,
                useRootMotionUnderWater = useRootMotionUnderWater,
                snapThreshold = snapThreshold,
            };
            Functions.StopMoveFunction();
        }

        public override void EntityStart()
        {
            Functions.EntityStart();
            CacheMotor.SetPosition(EntityTransform.position);
        }

        public override void ComponentOnEnable()
        {
            Functions.ComponentEnabled();
            CacheMotor.enabled = true;
            try
            {
                CacheMotor.SetPosition(EntityTransform.position);
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

        public override void EntityUpdate()
        {
#if UNITY_EDITOR
            Functions.stoppingDistance = stoppingDistance;
            Functions.movementSecure = movementSecure;
            Functions.jumpHeight = jumpHeight;
            Functions.applyJumpForceMode = applyJumpForceMode;
            Functions.applyJumpForceFixedDuration = applyJumpForceFixedDuration;
            Functions.backwardMoveSpeedRate = backwardMoveSpeedRate;
            Functions.gravity = gravity;
            Functions.maxFallVelocity = maxFallVelocity;
            Functions.airborneDelay = airborneDelay;
            Functions.doNotChangeVelocityWhileAirborne = doNotChangeVelocityWhileAirborne;
            Functions.landedPauseMovementDuration = landedPauseMovementDuration;
            Functions.beforeCrawlingPauseMovementDuration = beforeCrawlingPauseMovementDuration;
            Functions.afterCrawlingPauseMovementDuration = afterCrawlingPauseMovementDuration;
            Functions.underWaterThreshold = underWaterThreshold;
            Functions.autoSwimToSurface = autoSwimToSurface;
            Functions.alwaysUseRootMotion = alwaysUseRootMotion;
            Functions.dashingForceApplier = dashingForceApplier;
            Functions.useRootMotionForMovement = useRootMotionForMovement;
            Functions.useRootMotionForAirMovement = useRootMotionForAirMovement;
            Functions.useRootMotionForJump = useRootMotionForJump;
            Functions.useRootMotionForFall = useRootMotionForFall;
            Functions.useRootMotionUnderWater = useRootMotionUnderWater;
            Functions.snapThreshold = snapThreshold;
#endif
            float deltaTime = Time.deltaTime;
            Functions.UpdateRotation(deltaTime);
        }

        public void BeforeCharacterUpdate(float deltaTime)
        {
        }

        public void UpdateRotation(ref Quaternion currentRotation, float deltaTime)
        {

        }

        public void UpdateVelocity(ref Vector3 currentVelocity, float deltaTime)
        {
            Functions.UpdateMovement(deltaTime);
            currentVelocity = _motion / deltaTime;
        }

        public void AfterCharacterUpdate(float deltaTime)
        {
            Functions.AfterMovementUpdate(deltaTime);
            Functions.FixSwimUpPosition(deltaTime);
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

        public void SetPosition(Vector3 position)
        {
            CacheMotor.SetPosition(position);
        }

        public Bounds GetMovementBounds()
        {
            return CacheCapsuleCollider.bounds;
        }

        public void Move(Vector3 motion)
        {
            if (Functions.IsUnderWater && motion.y > 0)
                CacheMotor.ForceUnground();
            _motion = motion;
        }

        public void RotateY(float yAngle)
        {
            CacheMotor.SetRotation(Quaternion.Euler(0f, yAngle, 0f));
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

        public void SetLookRotation(Quaternion rotation, bool immediately)
        {
            Functions.SetLookRotation(rotation, immediately);
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
            if (CacheMotor.CharacterSweep(fromPosition + (Vector3.up * findDistance * 0.5f), EntityTransform.rotation, Vector3.down, findDistance, out RaycastHit closestHit, s_findGroundRaycastHits, CacheMotor.StableGroundLayers, QueryTriggerInteraction.Ignore) > 0)
            {
                result = closestHit.point;
                CacheMotor.SetPosition(fromPosition);
                return true;
            }
            result = fromPosition;
            CacheMotor.SetPosition(fromPosition);
            return false;
        }

        public void ApplyForce(ApplyMovementForceMode mode, Vector3 direction, ApplyMovementForceSourceType sourceType, int sourceDataId, int sourceLevel, float force, float deceleration, float duration)
        {
            Functions.ApplyForce(mode, direction, sourceType, sourceDataId, sourceLevel, force, deceleration, duration);
        }

        public EntityMovementForceApplier FindForceByActionKey(ApplyMovementForceSourceType sourceType, int sourceDataId)
        {
            return Functions.FindForceByActionKey(sourceType, sourceDataId);
        }

        public void ClearAllForces()
        {
            Functions.ClearAllForces();
        }

        public Vector3 GetSnapToGroundMotion(Vector3 motion, Vector3 platformMotion, Vector3 forceMotion)
        {
            return Vector3.zero;
        }
    }
}
