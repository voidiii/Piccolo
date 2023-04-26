#pragma once

#include "runtime/core/math/vector3.h"
#include "runtime/resource/res_type/components/rigid_body.h"
#include "runtime/resource/res_type/data/basic_shape.h"

namespace Piccolo
{
    enum SweepPass
    {
        SWEEP_PASS_UP,
        SWEEP_PASS_SIDE,
        SWEEP_PASS_DOWN,
        SWEEP_PASS_SENSOR
    };

    class Controller
    {
    public:
        virtual ~Controller() = default;

        virtual Vector3 move(const Vector3& current_position, const Vector3& displacement) = 0;
        virtual bool getGroundState() = 0;
    };

    class CharacterController : public Controller
    {
    public:
        CharacterController(const Capsule& capsule);
        ~CharacterController() = default;

        Vector3 move(const Vector3& current_position, const Vector3& displacement) override;
        bool getGroundState() { return m_is_touch_ground; };

    private:
        Capsule        m_capsule;
        Slide_Capsule  m_slide_capsule;
        RigidBodyShape m_rigidbody_shape;
        RigidBodyShape m_slide_test_shape;
        bool           m_is_touch_ground;
    };
} // namespace Piccolo
