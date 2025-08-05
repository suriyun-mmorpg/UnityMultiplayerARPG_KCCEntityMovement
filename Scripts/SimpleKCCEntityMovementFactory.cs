using KinematicCharacterController;
using UnityEngine;

namespace MultiplayerARPG
{
    public class SimpleKCCEntityMovementFactory : IEntityMovementFactory
    {
        public string Name => "Simple KCC Entity Movement";

        public DimensionType DimensionType => DimensionType.Dimension3D;

        public SimpleKCCEntityMovementFactory()
        {

        }

        public bool ValidateSourceObject(GameObject obj)
        {
            return true;
        }

        public IEntityMovementComponent Setup(GameObject obj, ref Bounds bounds)
        {
            bounds = default;
            MeshRenderer[] meshes = obj.GetComponentsInChildren<MeshRenderer>();
            for (int i = 0; i < meshes.Length; ++i)
            {
                if (i > 0)
                    bounds.Encapsulate(meshes[i].bounds);
                else
                    bounds = meshes[i].bounds;
            }

            SkinnedMeshRenderer[] skinnedMeshes = obj.GetComponentsInChildren<SkinnedMeshRenderer>();
            for (int i = 0; i < skinnedMeshes.Length; ++i)
            {
                if (i > 0)
                    bounds.Encapsulate(skinnedMeshes[i].bounds);
                else
                    bounds = skinnedMeshes[i].bounds;
            }

            float scale = Mathf.Max(obj.transform.localScale.x, obj.transform.localScale.y, obj.transform.localScale.z);
            bounds.size = bounds.size / scale;
            bounds.center = bounds.center / scale;

            obj.AddComponent<Rigidbody>();
            obj.AddComponent<CapsuleCollider>();

            KinematicCharacterMotor kccMotor = obj.AddComponent<KinematicCharacterMotor>();
            float height = bounds.size.y;
            float radius = Mathf.Min(bounds.extents.x, bounds.extents.z);
            kccMotor.SetCapsuleDimensions(radius, height, height * 0.5f);

            return obj.AddComponent<KCCEntityMovement>();
        }
    }
}
