#pragma once
#include "collision_grid.hpp"
#include "engine/common/utils.hpp"
#include "engine/common/math.hpp"


struct PhysicObject
{
    // Verlet
    Vec2 position      = {0.0f, 0.0f};
    Vec2 last_position = {0.0f, 0.0f};
    Vec2 acceleration  = {0.0f, 0.0f};
    Vec2 reference_position = {0.0f, 0.0f}; // Position mémorisée après stabilisation
    sf::Color color;
    float radius = 0.5f;

    PhysicObject() = default;

    explicit
    PhysicObject(Vec2 position_, float radius_ = 0.5f)
        : position(position_)
        , last_position(position_)
        , radius(radius_)
    {}

    void setPosition(Vec2 pos)
    {
        position      = pos;
        last_position = pos;
    }

    void update(float dt, float damping = 40.0f)
    {
        const Vec2 last_update_move = position - last_position;

        const Vec2 new_position = position + last_update_move + (acceleration - last_update_move * damping) * (dt * dt);
        last_position           = position;
        position                = new_position;
        acceleration = {0.0f, 0.0f};
    }

    void stop()
    {
        last_position = position;
    }

    void slowdown(float ratio)
    {
        last_position = last_position + ratio * (position - last_position);
    }

    [[nodiscard]]
    float getSpeed() const
    {
        return MathVec2::length(position - last_position);
    }

    [[nodiscard]]
    Vec2 getVelocity() const
    {
        return position - last_position;
    }

    void addVelocity(Vec2 v)
    {
        last_position -= v;
    }

    void setPositionSameSpeed(Vec2 new_position)
    {
        const Vec2 to_last = last_position - position;
        position           = new_position;
        last_position      = position + to_last;
    }

    void move(Vec2 v)
    {
        position += v;
    }
};
