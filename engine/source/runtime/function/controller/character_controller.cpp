#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"
#include "runtime/function/render/debugdraw/debug_draw_manager.h"
#include "runtime/function/framework/level/level_debugger.h"

namespace Piccolo
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PICCOLO_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_slide_test_shape                                   = RigidBodyShape();
        m_slide_test_shape.m_geometry                        = PICCOLO_REFLECTION_NEW(Slide_Capsule);
        *static_cast<Slide_Capsule*>(m_slide_test_shape.m_geometry) = m_slide_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;
        m_slide_test_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(Vector3(0, 0, capsule.m_half_height + capsule.m_radius), orientation, Vector3::UNIT_SCALE);
        
        m_slide_test_shape.m_local_transform =
            Transform(Vector3(0, 0, capsule.m_half_height + capsule.m_radius), orientation, Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        //DebugDrawGroup* debug_draw_group =
        //    g_runtime_global_context.m_debugdraw_manager->tryGetOrCreateDebugDrawGroup("hit physics");

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
            Vector3 step_direction = horizontal_direction;
            for(auto hit : hits) {
                // deal with steps
                // TODO: STAIR FUNCTION NOT DONE
                if(hit.hit_position.z - current_position.z > 0.00001f && hit.hit_position.z - current_position.z < 0.3f) {
                    step_height = hit.hit_position.z - current_position.z;
                    m_is_touch_ground = true;
                }
                // keep getting the projection along the planes with normals, normalise it for direction only
                // if the cos is less than zero that means the surface would not affect the slide 
                if(Math::cos(silde_direction.angleBetween(hit.hit_normal)) < -0.f) {
                    continue;
                }
                silde_direction = silde_direction.project(hit.hit_normal);
                silde_direction = Vector3(silde_direction.x, silde_direction.y, 0.f).normalisedCopy();
                // debug rendering
                //debug_draw_group->addLine(
                //            current_position, current_position + hit.hit_normal, Vector4(1.0f, 1.0f, 1.0f, 1.0f), Vector4(0.0f, 0.0f, 1.0f, 1.0f), 5.f);
            }
            Vector3 slide_displacement = silde_direction * displacement.dotProduct(silde_direction);
            hits.clear();
            bool stuck = false;
            if(physics_scene->sweep(
                m_rigidbody_shape,
                world_transform.getMatrix(),
                silde_direction,
                slide_displacement.length(),
                hits))
            {
                for(auto hit : hits) {
                    if (std::abs(hit.hit_normal.z) > 0.9f || std::abs(Math::cos(silde_direction.angleBetween(hit.hit_normal))) < 0.001f) {
                        // the ones that should not affect side movement, do nothing
                    } else {
                        stuck = true;
                        // debug rendering
                        //debug_draw_group->addLine(
                        //    current_position, current_position + slide_displacement * 100.0f, Vector4(1.0f, 1.0f, 1.0f, 1.0f), Vector4(1.0f, 1.0f, 1.0f, 1.0f), 5.f);
                    }
                }
            }
            if(!stuck) {
                final_position += slide_displacement;
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
