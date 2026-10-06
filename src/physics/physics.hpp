#pragma once
#include "collision_grid.hpp"
#include "physic_object.hpp"
#include "engine/common/utils.hpp"
#include "engine/common/index_vector.hpp"
#include "thread_pool/thread_pool.hpp"
#include "engine/common/color_utils.hpp"

struct PhysicSolver
{
    CIVector<PhysicObject> objects;
    CollisionGrid          grid;
    Vec2                   world_size;
    float                  cell_size; // Taille des cellules de la grille spatiale
    Vec2                   gravity = {0.0f, 50.0f};
    float                  wallPosition = 100.0f; // Position du mur
    float                  roof_y = 180.0f;       // Position du toit
    
    // Paramètres physiques
    float                  velocity_damping = 40.0f; // Amortissement de l'air
    float                  response_coef = 1.0f;     // Coefficient de rebond/réponse

    // Simulation solving pass count
    uint32_t        sub_steps;
    tp::ThreadPool& thread_pool;

    PhysicSolver(IVec2 size, float cell_size_, tp::ThreadPool& tp)
        : cell_size{cell_size_}
        , grid{static_cast<int32_t>(size.x / cell_size_) + 1, static_cast<int32_t>(size.y / cell_size_) + 1}
        , world_size{to<float>(size.x), to<float>(size.y)}
        , sub_steps{8}
        , thread_pool{tp}
    {
        grid.clear();
    }

    // Checks if two atoms are colliding and if so create a new contact
    void solveContact(uint32_t atom_1_idx, uint32_t atom_2_idx)
    {
        constexpr float eps = 0.0001f;
        PhysicObject& obj_1 = objects.data[atom_1_idx];
        PhysicObject& obj_2 = objects.data[atom_2_idx];
        const Vec2 o2_o1  = obj_1.position - obj_2.position;
        const float dist2 = o2_o1.x * o2_o1.x + o2_o1.y * o2_o1.y;
        const float min_dist = obj_1.radius + obj_2.radius;
        if (dist2 < min_dist * min_dist && dist2 > eps) {
            const float dist          = sqrt(dist2);
            // Mass ratio for realistic bounce based on area
            const float m1 = obj_1.radius * obj_1.radius;
            const float m2 = obj_2.radius * obj_2.radius;
            const float mass_ratio_1 = m1 / (m1 + m2);
            const float mass_ratio_2 = m2 / (m1 + m2);
            const float delta  = response_coef * (min_dist - dist);
            const Vec2 col_vec = (o2_o1 / dist) * delta;
            obj_1.position += col_vec * mass_ratio_2;
            obj_2.position -= col_vec * mass_ratio_1;
        }
    }

    void checkAtomCellCollisions(uint32_t atom_idx, const CollisionCell& c)
    {
        for (uint32_t i{0}; i < c.objects_count; ++i) {
            solveContact(atom_idx, c.objects[i]);
        }
    }

    void processCell(const CollisionCell& c, uint32_t index)
    {
        for (uint32_t i{0}; i < c.objects_count; ++i) {
            const uint32_t atom_idx = c.objects[i];
            checkAtomCellCollisions(atom_idx, grid.data[index - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index]);
            checkAtomCellCollisions(atom_idx, grid.data[index + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index + grid.height + 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height - 1]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height    ]);
            checkAtomCellCollisions(atom_idx, grid.data[index - grid.height + 1]);
        }
    }

    void solveCollisionThreaded(uint32_t start, uint32_t end)
    {
        for (uint32_t idx{start}; idx < end; ++idx) {
            processCell(grid.data[idx], idx);
        }
    }

    // Find colliding atoms
    void solveCollisions()
    {
        // Multi-thread grid
        const uint32_t thread_count = thread_pool.m_thread_count;
        const uint32_t slice_count  = thread_count * 2;
        const uint32_t slice_size   = (grid.width / slice_count) * grid.height;
        const uint32_t last_cell    = (2 * (thread_count - 1) + 2) * slice_size;
        // Find collisions in two passes to avoid data races

        // First collision pass
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                uint32_t const start{2 * i * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        // Eventually process rest if the world is not divisible by the thread count
        if (last_cell < grid.data.size()) {
            thread_pool.addTask([this, last_cell]{
                solveCollisionThreaded(last_cell, to<uint32_t>(grid.data.size()));
            });
        }
        thread_pool.waitForCompletion();
        // Second collision pass
        for (uint32_t i{0}; i < thread_count; ++i) {
            thread_pool.addTask([this, i, slice_size]{
                uint32_t const start{(2 * i + 1) * slice_size};
                uint32_t const end  {start + slice_size};
                solveCollisionThreaded(start, end);
            });
        }
        thread_pool.waitForCompletion();
    }

    // Add a new object to the solver
    uint64_t addObject(const PhysicObject& object)
    {
        return objects.push_back(object);
    }

    // Add a new object to the solver
    uint64_t createObject(Vec2 pos, float radius = 0.5f)
    {
        return objects.emplace_back(pos, radius);
    }

    void update(float dt)
    {
        // Perform the sub steps
        const float sub_dt = dt / static_cast<float>(sub_steps);
        for (uint32_t i(sub_steps); i--;) {
            addObjectsToGrid();
            solveCollisions();
            updateObjects_multi(sub_dt);
        }
    }

    void addObjectsToGrid()
    {
        grid.clear();
        // Safety border to avoid adding object outside the grid
        uint32_t i{0};
        for (const PhysicObject& obj : objects.data) {
            if (obj.position.x > 1.0f && obj.position.x < world_size.x - 1.0f &&
                obj.position.y > 1.0f && obj.position.y < world_size.y - 1.0f) {
                grid.addAtom(to<int32_t>(obj.position.x / cell_size), to<int32_t>(obj.position.y / cell_size), i);
            }
            ++i;
        }
    }

    void updateObjects_multi(float dt)
    {
        thread_pool.dispatch(to<uint32_t>(objects.size()), [&](uint32_t start, uint32_t end){
            for (uint32_t i{start}; i < end; ++i) {
                PhysicObject& obj = objects.data[i];
                // Add gravity
                obj.acceleration += gravity;
                Vec2 oldposition = obj.position;
                // Apply Verlet integration
                obj.update(dt, velocity_damping);
                
                // Apply map borders collisions
                const float margin = 5.0f;
                if (obj.position.x > world_size.x - margin) {
                    obj.position.x = world_size.x - margin;
                    obj.position.y = oldposition.y;
                } 
                // else if (obj.position.x < margin) {
                //     obj.position.x = margin;
                // }
                if (obj.position.y > world_size.y - margin) {
                    obj.position.y = world_size.y - margin;
                    obj.position.x = oldposition.x;
                } 
                // else if (obj.position.y < margin) {
                //     obj.position.y = margin;
                // }

                // Add wall collisions

                if (obj.position.x < wallPosition) {
                    obj.position.x = wallPosition; 
                }
                // Add roof collisions

                if (obj.position.y < roof_y ) {
                    obj.position.y = roof_y;
                    
                    
                }

                // obj.color = ColorUtils::getRainbow(obj.position.x * 0.01f);

            }
        });
    }
};
