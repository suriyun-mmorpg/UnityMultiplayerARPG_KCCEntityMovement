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
        public float groundedVerticalVelocity = -2f;
        public bool doNotChangeVelocityWhileAirborne;

        [Header("Pausing")]
        public float landedPauseMovementDuration = 0f;
        public float beforeCrawlingPauseMovementDuration = 0f;
        public float afterCrawlingPauseMovementDuration = 0f;

        [Header("Swimming")]
        [Range(0.1f, 1f)]
        public float underWaterThreshold = 0.75f;
        public bool autoSwimToSurface;

        [Header("Ground Checking")]
        public float groundCheckOffsets = 0.14f;
        public float groundCheckRadius = 0.28f;
        public Color groundCheckGizmosColor = Color.blue;

        [Header("Airborne Checking")]
        public float airborneCheckOffsets = 0.14f;
        public float airborneCheckRadius = 0.28f;
        public Color airborneCheckGizmosColor = Color.cyan;
        public float forceUngroundAfterJumpDuration = 0.1f;

        [Header("Crawl Checking")]
        public float crawlCheckOffsets = -0.14f;
        [Min(0)]
        public int crawlCheckRaycasts = 0;
        [Min(0f)]
        public float crawlCheckRadius = 1f;
        public Color crawlCheckGizmosColor = Color.yellow;

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
        public bool useRootMotionClimbing;
        public float rootMotionGroundedVerticalVelocity = -2f;

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

        protected float _forceUngroundCountdown = 0f;
        protected Vector3 _motion;
        protected Vector3 _internalVelocityAdd;
        protected int _allowToJumpOrDashCheckFrame = 0;
        protected bool _isAllowToJumpOrDash = true;
        protected float[] _crawlRaycastDegrees;

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
                groundedVerticalVelocity = groundedVerticalVelocity,
                doNotChangeVelocityWhileAirborne = doNotChangeVelocityWhileAirborne,
                landedPauseMovementDuration = landedPauseMovementDuration,
                beforeCrawlingPauseMovementDuration = beforeCrawlingPauseMovementDuration,
                afterCrawlingPauseMovementDuration = afterCrawlingPauseMovementDuration,
                underWaterThreshold = underWaterThreshold,
                autoSwimToSurface = autoSwimToSurface,
                dashingForceApplier = dashingForceApplier,
                alwaysUseRootMotion = alwaysUseRootMotion,
                useRootMotionForMovement = useRootMotionForMovement,
                useRootMotionForAirMovement = useRootMotionForAirMovement,
                useRootMotionForJump = useRootMotionForJump,
                useRootMotionForFall = useRootMotionForFall,
                useRootMotionUnderWater = useRootMotionUnderWater,
                useRootMotionClimbing = useRootMotionClimbing,
                rootMotionGroundedVerticalVelocity = rootMotionGroundedVerticalVelocity,
                snapThreshold = snapThreshold,
            };
            Functions.StopMoveFunction();
            _crawlRaycastDegrees = CalculateCrawlRaycastDegrees();
        }

        private float[] CalculateCrawlRaycastDegrees()
        {
            float[] result;
            if (crawlCheckRaycasts > 0)
            {
                result = new float[crawlCheckRaycasts];
                float increaseRaycastDegree = 360f / crawlCheckRaycasts;
                result = new float[crawlCheckRaycasts];
                for (int i = 0; i < crawlCheckRaycasts; ++i)
                {
                    result[i] = i * increaseRaycastDegree;
                }
                return result;
            }
            return null;
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
            if (_forceUngroundCountdown > 0f)
                _forceUngroundCountdown -= deltaTime;
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
            currentVelocity = _motion;
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
            if (_forceUngroundCountdown > 0f)
                return false;
            return CacheMotor.GroundingStatus.IsStableOnGround;
        }

        public bool AirborneCheck()
        {
            if (_forceUngroundCountdown > 0f)
                return true;
            return !Physics.CheckSphere(GetAirborneCheckCenter(), airborneCheckRadius, CacheMotor.StableGroundLayers, QueryTriggerInteraction.Ignore);
        }

        public virtual bool AllowToJump()
        {
            return AllowToJumpOrDash();
        }

        public virtual bool AllowToDash()
        {
            return AllowToJumpOrDash();
        }

        protected virtual bool AllowToJumpOrDash()
        {
            if (Time.frameCount == _allowToJumpOrDashCheckFrame)
                return _isAllowToJumpOrDash;
            _allowToJumpOrDashCheckFrame = Time.frameCount;
            _isAllowToJumpOrDash = Physics.CheckSphere(GetGroundCheckCenter(), groundCheckRadius, CacheMotor.StableGroundLayers, QueryTriggerInteraction.Ignore);
            return _isAllowToJumpOrDash;
        }

        public bool AllowToCrouch()
        {
            return true;
        }

        public bool AllowToCrawl()
        {
            if (crawlCheckRaycasts == 0f)
                return true;
            Vector3 raycastOrigin = GetCrawlCheckCenter();
            for (int i = 0; i < crawlCheckRaycasts; ++i)
            {
                int hitCount = Physics.RaycastNonAlloc(raycastOrigin, Quaternion.Euler(0f, _crawlRaycastDegrees[i], 0f) * EntityTransform.forward, s_findGroundRaycastHits, crawlCheckRadius, CacheMotor.StableGroundLayers, QueryTriggerInteraction.Ignore);
                if (hitCount > 0)
                    return false;
            }
            return true;
        }

        private Vector3 AdjustCrawlMotion(MovementState movementState, ExtraMovementState extraMovementState, Vector3 motion)
        {
            if (extraMovementState != ExtraMovementState.IsCrawling || crawlCheckRaycasts == 0 || !IsClient)
                return motion;
            Vector3 raycastOrigin = GetCrawlCheckCenter();
            Vector3 moveDirection = motion.GetXZ().normalized;
            float nearestHitDistance = float.MaxValue;
            float nearestRaycastAngle = 0f;
            RaycastHit? nearestHit = null;
            for (int i = 0; i < crawlCheckRaycasts; ++i)
            {
                Vector3 raycastDirection = Quaternion.Euler(0f, _crawlRaycastDegrees[i], 0f) * moveDirection;
                float raycastAngle = Vector3.Angle(moveDirection, raycastDirection);
                if (raycastAngle > 90f)
                    continue;
                int hitCount = Physics.RaycastNonAlloc(raycastOrigin, raycastDirection, s_findGroundRaycastHits, crawlCheckRadius, CacheMotor.StableGroundLayers, QueryTriggerInteraction.Ignore);
                if (hitCount <= 0)
                    continue;
                for (int j = 0; j < hitCount; ++j)
                {
                    RaycastHit hit = s_findGroundRaycastHits[j];
                    if (hit.distance >= nearestHitDistance)
                        continue;
                    nearestRaycastAngle = raycastAngle;
                    nearestHitDistance = hit.distance;
                    nearestHit = hit;
                }
            }
            if (nearestHit.HasValue)
            {
                Vector3 hitNormal = nearestHit.Value.normal;
                if (nearestHit.Value.distance < crawlCheckRadius * 0.9f)
                    return motion + hitNormal * crawlCheckRadius;
                return Vector3.ProjectOnPlane(motion, hitNormal);
            }
            return motion;
        }

        protected virtual Vector3 GetGroundCheckCenter()
        {
            return new Vector3(EntityTransform.position.x, EntityTransform.position.y - groundCheckOffsets, EntityTransform.position.z);
        }

        protected virtual Vector3 GetAirborneCheckCenter()
        {
            return new Vector3(EntityTransform.position.x, EntityTransform.position.y - airborneCheckOffsets, EntityTransform.position.z);
        }

        protected virtual Vector3 GetCrawlCheckCenter()
        {
            return new Vector3(EntityTransform.position.x, EntityTransform.position.y - crawlCheckOffsets, EntityTransform.position.z);
        }

#if UNITY_EDITOR
        protected virtual void OnDrawGizmos()
        {
            Color prevColor = Gizmos.color;
            Gizmos.color = groundCheckGizmosColor;
            Gizmos.DrawWireSphere(GetGroundCheckCenter(), groundCheckRadius);
            Gizmos.color = airborneCheckGizmosColor;
            Gizmos.DrawWireSphere(GetAirborneCheckCenter(), airborneCheckRadius);
            if (crawlCheckRaycasts > 0)
            {
                Gizmos.color = crawlCheckGizmosColor;
                if (_crawlRaycastDegrees == null)
                    _crawlRaycastDegrees = CalculateCrawlRaycastDegrees();
                for (int i = 0; i < crawlCheckRaycasts; ++i)
                {
                    Gizmos.DrawLine(GetCrawlCheckCenter(), GetCrawlCheckCenter() + Quaternion.Euler(0f, _crawlRaycastDegrees[i], 0f) * EntityTransform.forward * crawlCheckRadius);
                }
            }
            Gizmos.color = prevColor;
        }
#endif

        public void SetPosition(Vector3 position)
        {
            CacheMotor.SetPosition(position);
        }

        public Bounds GetMovementBounds()
        {
            return CacheCapsuleCollider.bounds;
        }

        public void Move(MovementState movementState, ExtraMovementState extraMovementState, Vector3 motion, float deltaTime)
        {
            if (Functions.IsUnderWater && motion.y > 0)
                CacheMotor.ForceUnground();
            _motion = AdjustCrawlMotion(movementState, extraMovementState, motion / deltaTime);
        }

        public void RotateY(float yAngle)
        {
            CacheMotor.SetRotation(Quaternion.Euler(0f, yAngle, 0f));
        }

        public void OnJumpForceApplied(float verticalVelocity)
        {
            _forceUngroundCountdown = forceUngroundAfterJumpDuration;
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
