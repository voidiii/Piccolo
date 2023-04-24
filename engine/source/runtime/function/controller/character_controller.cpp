#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Piccolo
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PICCOLO_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(Vector3(0, 0, capsule.m_half_height + capsule.m_radius), orientation, Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        std::shared_ptr<PhysicsScene> physics_scene =
            g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
        ASSERT(physics_scene);

        std::vector<PhysicsHitInfo> hits;

        Transform world_transform = Transform(
            current_position + 0.1f * Vector3::UNIT_Z,
            Quaternion::IDENTITY,
            Vector3::UNIT_SCALE);

        Vector3 vertical_displacement = displacement.z * Vector3::UNIT_Z;
        Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);
        Vector3 vertical_direction = vertical_displacement.normalisedCopy();
        Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();
        Vector3 displacement_direction = displacement.normalisedCopy();

        Vector3 final_position = current_position;

        m_is_touch_ground = physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            Vector3::NEGATIVE_UNIT_Z,
            0.105f,
            hits);
        
        hits.clear();

        world_transform.m_position -= 0.1f * Vector3::UNIT_Z;

        // vertical pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            vertical_direction,
            vertical_displacement.length(),
            hits))
        {
            final_position += hits[0].hit_distance * vertical_direction;
        }
        else
        {
            final_position += vertical_displacement;
        }
        hits.clear();

        if(physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            horizontal_direction,
            horizontal_displacement.length(),
            hits))
        {
            Vector3 silde_direction = horizontal_direction;
            bool stuck = false;
            //float step_height = -0.5f;
            for(auto hit : hits) {
                // keep getting the projection along the planes with normals, normalise it for direction only
                silde_direction = silde_direction.project(hit.hit_normal);
                silde_direction = Vector3(silde_direction.x, silde_direction.y, 0.f).normalisedCopy();
            }
            Vector3 slide_displacement = silde_direction * displacement.dotProduct(silde_direction);
            hits.clear();
            if(!physics_scene->sweep(
                m_rigidbody_shape,
                world_transform.getMatrix(),
                silde_direction,
                slide_displacement.length(),
                hits))
            {
                final_position += slide_displacement;
            } else {
                for(auto hit : hits) {
                    if (std::abs(hit.hit_normal.z) > 0.9f || std::abs(Math::cos(silde_direction.angleBetween(hit.hit_normal))) < 0.001f) {
                        final_position += slide_displacement;
                    }
                }
            }
        }
        else
        {
            final_position += horizontal_displacement;
        }
        hits.clear();
        
        return final_position;
    }
} // namespace Piccolo
