#include <iostream>
#include <fstream>

#include "engine/window_context_handler.hpp"
#include "engine/common/color_utils.hpp"
#include "engine/common/number_generator.hpp"

#include "physics/physics.hpp"
#include "thread_pool/thread_pool.hpp"
#include "renderer/renderer.hpp"

void save_state(const PhysicSolver& solver, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return;
    uint32_t count = solver.objects.size();
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (uint32_t i = 0; i < count; i++) {
        out.write(reinterpret_cast<const char*>(&solver.objects.data[i]), sizeof(PhysicObject));
    }
}

bool load_state(PhysicSolver& solver, const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    uint32_t count;
    if (!in.read(reinterpret_cast<char*>(&count), sizeof(count))) return false;
    for (uint32_t i = 0; i < count; i++) {
        PhysicObject obj;
        in.read(reinterpret_cast<char*>(&obj), sizeof(PhysicObject));
        solver.addObject(obj);
    }
    return true;
}

int main()
{
    // ==========================================
    // CONFIGURATION GENERALE
    // ==========================================

    // Taille de la fenêtre graphique affichée à l'écran (pixels)
    const uint32_t window_width = 1920;
    const uint32_t window_height = 1000;

    // Dimensions de l'espace de simulation physique (unités arbitraires)
    const IVec2 simulation_size{460, 540};

    // Configuration des particules
    const uint32_t nbObjects = 9000;
    const uint32_t particles_spawn_per_frame = 50; 
    const float particle_radius_min = 0.2f; 
    const float particle_radius_max = 3.f; 
    const float stabilization_time = 10.0f; 

    // Configuration de l'environnement
    const float wall_start_position = 0.0f; 
    const float roof_y = 180.0f;              

    // Configuration de la fondation
    const float foundation_width = 120.0f;
    const float foundation_start_y = 270.0f; 
    const float foundation_speed = 0.15f;     

    // Configuration de la physique
    const float physics_gravity = 50.f;      
    const float physics_damping = 40.0f;      
    const float physics_response_coef = 1.0f; 
    const float physics_friction_coef = 0.3f; // Frottement inter-particulaire (0.0 lisse, >0 rugueux)
    const uint32_t physics_sub_steps = 16;     // Precision du solveur (plus c'est eleve, plus le sol est rigide/dilatant)
    const float grid_cell_size = 8.0f; 

    // Position du point d'apparition (robinet)
    // On tombe au milieu
    const float spawn_x = simulation_size.x / 2.0f;
    const float spawn_y = roof_y + 10.0f; // Juste sous le toit

    // Fichier de sauvegarde de l'état
    const std::string save_filename = "foundation_state.bin";
    const bool use_save_file = true;

    // ==========================================
    // INITIALISATION DU MOTEUR
    // ==========================================

    WindowContextHandler app("Verlet-MultiThread - Fondation", sf::Vector2u(window_width, window_height), sf::Style::Default);
    RenderContext &render_context = app.getRenderContext();

    tp::ThreadPool thread_pool(10);
    PhysicSolver solver{simulation_size, grid_cell_size, thread_pool};
    solver.wallPosition = wall_start_position;
    solver.roof_y = roof_y;
    solver.gravity = {0.0f, physics_gravity};
    solver.velocity_damping = physics_damping;
    solver.response_coef = physics_response_coef;
    solver.friction_coef = physics_friction_coef;
    solver.sub_steps = physics_sub_steps;
    
    // Initialisation de la fondation (désactivée pendant le remplissage)
    solver.use_foundation = false;
    solver.foundation_x_min = (simulation_size.x - foundation_width) / 2.0f;
    solver.foundation_x_max = (simulation_size.x + foundation_width) / 2.0f;
    solver.foundation_y = foundation_start_y;

    Renderer renderer(solver, thread_pool);

    const float margin = 1.0f;
    const auto zoom = static_cast<float>(window_height - margin) / static_cast<float>(simulation_size.y);
    render_context.setZoom(zoom);
    render_context.setFocus({simulation_size.x * 0.5f, simulation_size.y * 0.5f});

    bool emit = true;
    app.getEventManager().addKeyPressedCallback(sf::Keyboard::Space, [&](sfev::CstEv)
                                                { emit = !emit; });

    constexpr uint32_t fps_cap = 60;
    int32_t target_fps = fps_cap;
    app.getEventManager().addKeyPressedCallback(sf::Keyboard::S, [&](sfev::CstEv)
                                                {
        target_fps = target_fps ? 0 : fps_cap;
        app.setFramerateLimit(target_fps); });

    // Main loop
    const float dt = 1.0f / static_cast<float>(fps_cap);
    float time = 0.0f;

    if (use_save_file && load_state(solver, save_filename)) {
        std::cout << "Etat initial charge depuis " << save_filename << std::endl;
        // On n'a plus besoin d'emettre
        emit = false;
        
        // Mettre à jour la grille car les objets ont été chargés
        solver.addObjectsToGrid();
    } else {
        std::cout << "Demarrage de l'emission et de la stabilisation..." << std::endl;
        
        //EMISSION DES PARTICULES
        while (emit)
        {
            if (solver.objects.size() < nbObjects)
            {
                for (uint32_t i = 0; i < particles_spawn_per_frame; ++i)
                {
                    if (solver.objects.size() >= nbObjects) break;

                    const float random_radius = RNGf::getRange(particle_radius_min, particle_radius_max);
                    
                    float final_spawn_x = RNGf::getRange(20.0f, simulation_size.x - 20.0f);
                    float final_spawn_y = spawn_y + RNGf::getRange(-5.0f, 5.0f);

                    const auto id = solver.createObject({final_spawn_x, final_spawn_y}, random_radius);
                    
                    // Vitesse purement verticale vers le bas
                    solver.objects[id].last_position.y -= RNGf::getRange(0.0f, 0.5f);
                    // On met une vitesse horizontale quasi-nulle pour tomber bien droit
                    solver.objects[id].last_position.x -= RNGf::getRange(-0.1f, 0.1f);
                }
            }
            else
            {
                emit = false;
            }
            
            solver.update(dt);
            render_context.clear();
            renderer.render(render_context);
            render_context.display();
        }

        //PHASE DE STABILISATION
        while (time < stabilization_time)
        {
            time += dt;
            solver.update(dt);
            render_context.clear();
            renderer.render(render_context);
            render_context.display();
        }

        // Sauvegarde des positions de référence après stabilisation
        for (auto& obj : solver.objects.data) {
            obj.reference_position = obj.position;
        }

        if (use_save_file) {
            save_state(solver, save_filename);
            std::cout << "Etat initial sauvegarde dans " << save_filename << std::endl;
        }
    }

    // ACTIVATION DE LA FONDATION APRES REMPLISSAGE
    solver.use_foundation = true;
    
    // Placer la fondation juste au-dessus des particules sous son emprise
    float highest_y = static_cast<float>(simulation_size.y);
    for (const auto& obj : solver.objects.data) {
        if (obj.position.x >= solver.foundation_x_min && obj.position.x <= solver.foundation_x_max) {
            float top_y = obj.position.y - obj.radius;
            if (top_y < highest_y) {
                highest_y = top_y;
            }
        }
    }
    if (highest_y < simulation_size.y) {
        solver.foundation_y = highest_y - 1.0f; // On la place juste au-dessus
    }

    // DEPLACEMENT DE LA FONDATION
    while (app.run())
    {
        time += dt;
        solver.foundation_y += foundation_speed; // La fondation descend
        solver.update(dt);
        render_context.clear();
        renderer.render(render_context);
        render_context.display();
    }

    return 0;
}
